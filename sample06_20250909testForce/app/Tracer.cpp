#include "Tracer.h"
#include <stdio.h>
#include <cstdlib> // abs関数のため

// etrobo_tr方式の定数定義
const float Tracer::Kp = 0.8;  // 比例定数（オーバーシュート防止）
const float Tracer::Kd = 0.2;  // 微分定数（変化率抑制）
const int Tracer::bias = 0;    // バイアス
const int Tracer::target = 25; // 目標値（黒と白の中間値）

// 青色検知用定数定義
const int Tracer::BLUE_THRESHOLD = 120;      // 青色判定閾値
const int Tracer::COLOR_DIFF_THRESHOLD = 50; // 他色との差の閾値

// 前進制御用定数
const float Tracer::WHEEL_DIAMETER_CM = 5.4f; // ホイール直径（実機に合わせて調整）

Tracer::Tracer() : leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true),
                   rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true),
                   colorSensor(EPort::PORT_E),
                   
                   mPreviousError(0),
                   mIsInitialized(false),
                   mLineTraceEnabled(true),              // デフォルトでライントレース有効
                   mBlueDetectionEnabled(true),          // デフォルトで青色検知有効
                   mBlueDetectionCount(0),               // 青色検知回数初期化
                   mCurrentBaseSpeed(DEFAULT_BASE_SPEED), // 初期速度設定
                   mIsStopped(false),                     // 停止フラグ初期化
                   mInitialSequenceCompleted(false),     // 初期処理未完了
                   mInitialStartTime(0)                   // 初期処理開始時刻初期化
{
}

void Tracer::init()
{
  leftWheel.resetCount();
  rightWheel.resetCount();
  mIsInitialized = true;
}

void Tracer::terminate()
{
  leftWheel.stop();
  rightWheel.stop();
}

void Tracer::run()
{
  if (!mIsInitialized)
  {
    init();
  }

  // 初期処理が未完了の場合は初期処理を実行
  if (!mInitialSequenceCompleted)
  {
    performInitialSequence();
    return;
  }

  // 完全停止フラグチェック
  if (mIsStopped) {
    leftWheel.stop();
    rightWheel.stop();
    return; // 停止状態を維持
  }

  // 青色検知チェック
  if (mBlueDetectionEnabled && detectBlue())
  {
    mBlueDetectionCount++; // 検知回数をカウント
    printf("青色を検知しました! 回数: %d\n", mBlueDetectionCount);

    // 1回目の青色検知後に速度を下げる
    if (mBlueDetectionCount == 1)
    {
      setSlowMode(true);
      printf("1回目の青色検知後、低速モードに切り替えました\n");
    }

    // 青色検知を無効にする（処理中の重複防止）
    setBlueDetectionEnabled(false);

    // ライントレースを無効にする
    setLineTraceEnabled(false);

    // 検知回数に応じた動作実行
    executeBlueAction();

    // 完全停止が設定されていない場合のみライントレースを再開
    if (!mIsStopped) {
      // 青色検知とライントレースを再び有効にする
      setBlueDetectionEnabled(true);
      setLineTraceEnabled(true);
      printf("ライントレース再開\n");
    } else {
      printf("完全停止状態を維持\n");
    }

    return; // 青色検知時は処理を終了
  }

  // ライントレースが無効の場合は処理をスキップ
  if (!mLineTraceEnabled)
  {
    return;
  }

  // etrobo_tr方式のライントレース処理
  int diffReflection = calDiffReflection();

  // PD制御による操作量計算
  float turn = calcPropValue(diffReflection);

  // 適応的速度計算
  int adaptiveSpeed = calcAdaptiveSpeed(turn);

  // モーター制御
  int pwm_l = adaptiveSpeed - turn;
  int pwm_r = adaptiveSpeed + turn;
  leftWheel.setPower(pwm_l);
  rightWheel.setPower(pwm_r);
}

/**
 * 反射光の差分を計算する
 * @return ライン境界とセンサ値との差分
 */
int Tracer::calDiffReflection() const
{
  int diff = colorSensor.getReflection() - target;
  return diff;
}

/**
 * PD制御による操作量を計算する
 * @param diffReflection ライン境界との差分
 * @return 操作量
 */
float Tracer::calcPropValue(int diffReflection)
{
  // P制御（比例制御）
  float pTerm = Kp * diffReflection;

  // D制御（微分制御）- オーバーシュートを防ぐ
  float dTerm = Kd * (diffReflection - mPreviousError);
  mPreviousError = diffReflection;

  float turn = pTerm + dTerm + bias;

  return turn;
}

/**
 * 適応的速度を計算する（etrobo_tr方式）
 * @param turn 旋回量
 * @return 適応的速度
 */
int Tracer::calcAdaptiveSpeed(float turn)
{
  // 旋回量の絶対値が大きいほど速度を下げる
  float turnAbs = abs(turn);

  // 旋回量に応じた速度制御
  if (turnAbs > 25)
  {
    // 急カーブ: 大幅減速
    return MIN_SPEED;
  }
  else if (turnAbs > 15)
  {
    // 中程度のカーブ: 中程度減速（50%）
    return (mCurrentBaseSpeed * 50) / 100;
  }
  else if (turnAbs > 8)
  {
    // 軽いカーブ: 軽度減速（70%）
    return (mCurrentBaseSpeed * 70) / 100;
  }
  else
  {
    // 直線: 現在の基本速度
    return mCurrentBaseSpeed;
  }
}

/**
 * 青色を検知する（RGB値で判定）- etrobo_tr方式
 * @retval true 青色検知 / false 青色なし
 */
bool Tracer::detectBlue() const
{
  spikeapi::ColorSensor::RGB rgb;
  colorSensor.getRGB(rgb);

  // 青色の条件：
  // 1. 青の値が閾値以上
  // 2. 青が赤より一定以上大きい
  // 3. 青が緑より一定以上大きい
  bool isBlue = (rgb.b > BLUE_THRESHOLD) &&
                (rgb.b > rgb.r + COLOR_DIFF_THRESHOLD) &&
                (rgb.b > rgb.g + COLOR_DIFF_THRESHOLD);

  return isBlue;
}

/**
 * 指定距離を前進する
 * @param distanceCm 前進距離（cm）
 */
void Tracer::moveForward(float distanceCm, TurnDirection direction, float turnIntensity)
{
  const char *dirName = (direction == TurnDirection::LEFT) ? "LEFT" : (direction == TurnDirection::RIGHT) ? "RIGHT"
                                                                                                          : "STRAIGHT";

  // 開始時のエンコーダ値を個別に記録
  int32_t leftStartCount = leftWheel.getCount();
  int32_t rightStartCount = rightWheel.getCount();

  // 基本距離をエンコーダ角度に変換
  float circumference = 3.14159265f * WHEEL_DIAMETER_CM;
  float baseRotations = distanceCm / circumference;
  int32_t baseDegrees = (int32_t)(baseRotations * 360.0f);

  // 曲がり方向に応じて左右の目標距離を調整（turnIntensityで強度調整）
  int32_t leftTargetDegrees = baseDegrees;
  int32_t rightTargetDegrees = baseDegrees;

  if (direction == TurnDirection::LEFT)
  {
    // 左曲がり：右ホイールをより多く回転（turnIntensityで調整）
    float distanceMultiplier = 1.0f + (turnIntensity * 0.2f); // 基本+ turnIntensity×20%
    rightTargetDegrees = (int32_t)(baseDegrees * distanceMultiplier);
    printf("左曲がり - 距離倍率: %.2f, 右目標: %d度\n", distanceMultiplier, rightTargetDegrees);
  }
  else if (direction == TurnDirection::RIGHT)
  {
    // 右曲がり：左ホイールをより多く回転（turnIntensityで調整）
    float distanceMultiplier = 1.0f + (turnIntensity * 0.2f); // 基本+ turnIntensity×20%
    leftTargetDegrees = (int32_t)(baseDegrees * distanceMultiplier);
    printf("右曲がり - 距離倍率: %.2f, 左目標: %d度\n", distanceMultiplier, leftTargetDegrees);
  }

  int32_t leftTargetCount = leftStartCount + leftTargetDegrees;
  int32_t rightTargetCount = rightStartCount + rightTargetDegrees;

  // turnIntensityに応じた速度差をつけて曲がる強度を調整
  int leftPower = mCurrentBaseSpeed;
  int rightPower = mCurrentBaseSpeed;

  if (direction == TurnDirection::LEFT)
  {
    // 左曲がり：左ホイールを遅くする（turnIntensityで調整）
    float speedReduction = 0.1f + (turnIntensity * 0.1f); // 基本10%減速 + turnIntensity×10%
    if (speedReduction > 0.8f) speedReduction = 0.8f;     // 最大80%減速まで
    leftPower = (int)(mCurrentBaseSpeed * (1.0f - speedReduction));
    if (leftPower < 5) leftPower = 5; // 最低速度保証
    printf("左曲がり - 減速率: %.1f%%, 左速度: %d\n", speedReduction * 100, leftPower);
  }
  else if (direction == TurnDirection::RIGHT)
  {
    // 右曲がり：右ホイールを遅くする（turnIntensityで調整）
    float speedReduction = 0.1f + (turnIntensity * 0.1f); // 基本10%減速 + turnIntensity×10%
    if (speedReduction > 0.8f) speedReduction = 0.8f;     // 最大80%減速まで
    rightPower = (int)(mCurrentBaseSpeed * (1.0f - speedReduction));
    if (rightPower < 5) rightPower = 5; // 最低速度保証
    printf("右曲がり - 減速率: %.1f%%, 右速度: %d\n", speedReduction * 100, rightPower);
  }

  // 前進開始（調整された速度差で）
  leftWheel.setPower(leftPower);
  rightWheel.setPower(rightPower);

  // 平均距離で判定（スムーズな停止）
  int32_t averageTarget = (leftTargetCount + rightTargetCount) / 2;

  while (true)
  {
    int32_t leftCurrentCount = leftWheel.getCount();
    int32_t rightCurrentCount = rightWheel.getCount();
    int32_t averageCurrent = (leftCurrentCount + rightCurrentCount) / 2;

    // 平均距離で判定
    if (averageCurrent >= averageTarget)
    {
      break;
    }
  }

  // 最終的に両方停止
  leftWheel.stop();
  rightWheel.stop();
}

/**
 * ライントレース有効/無効を設定する
 * @param enabled true=ライントレース有効, false=無効
 */
void Tracer::setLineTraceEnabled(bool enabled)
{
  mLineTraceEnabled = enabled;
}

/**
 * ライントレースの状態を取得する
 * @return true=ライントレース有効, false=無効
 */
bool Tracer::isLineTraceEnabled() const
{
  return mLineTraceEnabled;
}

/**
 * 青色検知有効/無効を設定する
 * @param enabled true=青色検知有効, false=無効
 */
void Tracer::setBlueDetectionEnabled(bool enabled)
{
  mBlueDetectionEnabled = enabled;
}

/**
 * 青色検知の状態を取得する
 * @return true=青色検知有効, false=無効
 */
bool Tracer::isBlueDetectionEnabled() const
{
  return mBlueDetectionEnabled;
}

/**
 * 青色検知時の動作実行（検知回数に応じた処理）
 */
void Tracer::executeBlueAction()
{
  switch (mBlueDetectionCount)
  {
  case 1: // 1回目の青色検知
    moveForward(13, TurnDirection::STRAIGHT, 0.0f);
    moveForward(10, TurnDirection::RIGHT, 1.3f);
    break;

  case 2: // 2回目の青色検知
    moveForward(10, TurnDirection::RIGHT, 2.0f);
    setCompleteStop(true);  // 完全停止フラグを設定
    moveForward(3, TurnDirection::LEFT, 10.0f);
    printf("2回目の青色検知完了 - 完全停止します\n");
    setCompleteStop(true);  // 完全停止フラグを設定
    break;

  case 3: // 3回目の青色検知
    moveForward(10, TurnDirection::RIGHT, 0.2f);
    moveForward(10, TurnDirection::LEFT, 0.2f);
    moveForward(10, TurnDirection::STRAIGHT, 0.0f);
    break;

  case 4: // 4回目の青色検知
    moveForward(5, TurnDirection::STRAIGHT, 0.0f);
    moveForward(20, TurnDirection::LEFT, 0.8f);
    moveForward(20, TurnDirection::LEFT, 0.8f);
    break;
  }
}

/**
 * 動作安定化待機（モーター停止の完了を確実にする）
 */
void Tracer::waitForStabilization()
{
  // 短時間待機してモーターの完全停止を確実にする
  for (volatile int i = 0; i < 50000; i++)
    ;
  printf("動作安定化待機完了\n");
}

/**
 * 低速モード設定
 * @param enabled true=低速モード, false=通常速度
 */
void Tracer::setSlowMode(bool enabled)
{
  if (enabled)
  {
    mCurrentBaseSpeed = SLOW_BASE_SPEED;
    printf("低速モード有効: 速度 %d\n", mCurrentBaseSpeed);
  }
  else
  {
    mCurrentBaseSpeed = DEFAULT_BASE_SPEED;
    printf("通常速度モード: 速度 %d\n", mCurrentBaseSpeed);
  }
}

/**
 * 現在の基本速度取得
 * @return 現在の基本速度
 */
int Tracer::getCurrentBaseSpeed() const
{
  return mCurrentBaseSpeed;
}

/**
 * 完全停止設定
 * @param stopped true=完全停止, false=動作継続
 */
void Tracer::setCompleteStop(bool stopped)
{
  mIsStopped = stopped;
  if (stopped) {
    leftWheel.stop();
    rightWheel.stop();
    printf("完全停止モード有効\n");
  } else {
    printf("動作継続モード\n");
  }
}

/**
 * 停止状態取得
 * @return true=停止中, false=動作中
 */
bool Tracer::isStopped() const
{
  return mIsStopped;
}

/**
 * 初期処理実行
 * ①前進を2秒ほどする
 * ②moveForward(2, TurnDirection::RIGHT, 5.0f);
 * ③moveForward(2, TurnDirection::LEFT, 5.0f);
 * ④カラーセンサが黒を検知したら走行停止
 */
void Tracer::performInitialSequence()
{
  static int sequenceStep = 0;
  static unsigned long stepStartTime = 0;
  static bool firstRun = true;
  
  if (firstRun) {
    printf("初期処理を開始します\n");
    mInitialStartTime = 0; // 簡易的な時間管理（実装依存）
    sequenceStep = 0;
    stepStartTime = 0;
    firstRun = false;
  }
  
  // 簡易的な時間カウンタ（runメソッドの呼び出し回数で概算）
  static unsigned long timeCounter = 0;
  timeCounter++;
  
  switch (sequenceStep) {
    case 0: // ①前進を2秒ほどする
      printf("ステップ1: 2秒間前進開始\n");
      leftWheel.setPower(mCurrentBaseSpeed);
      rightWheel.setPower(mCurrentBaseSpeed);
      stepStartTime = timeCounter;
      sequenceStep = 1;
      break;

    case 1: // 2秒間の前進継続チェック
      // 約2秒（100ms周期で20回＝2秒と仮定）
      if (timeCounter - stepStartTime >= 20) {
        leftWheel.stop();
        rightWheel.stop();
        printf("ステップ1完了: 2秒間前進終了\n");
        sequenceStep = 2;
        stepStartTime = timeCounter;
      } else {
        leftWheel.setPower(mCurrentBaseSpeed);
        rightWheel.setPower(mCurrentBaseSpeed);
      }
      break;

    case 2: // 安定化待機後、右カーブ移動
      if (timeCounter - stepStartTime >= 5) { // 500ms待機
        printf("ステップ2: 右カーブ移動開始 (4cm, 強度3.0)\n");
        moveForward(6, TurnDirection::RIGHT, 3.0f);
        printf("ステップ2完了: 右カーブ移動終了\n");
        sequenceStep = 3;
        stepStartTime = timeCounter;
      }
      break;

    case 3: // 直線走行10cm
      printf("ステップ3: 直線走行開始 (10cm)\n");
      moveForward(20, TurnDirection::STRAIGHT, 0.0f);
      printf("ステップ3完了: 直線走行終了\n");
      sequenceStep = 4;
      stepStartTime = timeCounter;
      break;

    case 4: // 安定化待機後、左カーブ移動
      if (timeCounter - stepStartTime >= 5) { // 500ms待機
        printf("ステップ4: 左カーブ移動開始 (7cm, 強度3.0)\n");
        moveForward(7, TurnDirection::LEFT, 3.0f);
        printf("ステップ4完了: 左カーブ移動終了\n");
        sequenceStep = 5;
      }
      break;

    case 5: // ⑤カラーセンサが黒を検知するまで待機
      if (detectBlack()) {
        leftWheel.stop();
        rightWheel.stop();
        printf("ステップ5完了: 黒色を検知しました。初期処理完了\n");
        mInitialSequenceCompleted = true;
        sequenceStep = 0; // リセット
        timeCounter = 0;  // リセット
        firstRun = true;  // リセット
      } else {
        leftWheel.setPower(SLOW_BASE_SPEED);
        rightWheel.setPower(SLOW_BASE_SPEED);
      }
      break;
  }
}

/**
 * 黒色検知メソッド
 * @retval true 黒色検知 / false 黒色なし
 */
bool Tracer::detectBlack() const
{
  int reflection = colorSensor.getReflection();
  // 黒色の判定閾値（通常10以下が黒色）
  const int BLACK_THRESHOLD = 15;
  return reflection < BLACK_THRESHOLD;
}

/**
 * 初期処理完了状態取得
 * @return true=初期処理完了, false=初期処理未完了
 */
bool Tracer::isInitialSequenceCompleted() const
{
  return mInitialSequenceCompleted;
}
