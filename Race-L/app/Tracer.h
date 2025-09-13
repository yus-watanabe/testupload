#include "Motor.h"
#include "ColorSensor.h"

using namespace spikeapi;

class Tracer {
public:
  Tracer();
  void run();
  void init();
  void terminate();
  
  // 初期処理状態確認用（public）
  bool isInitialSequenceCompleted() const;   // 初期処理完了状態取得

private:
  Motor leftWheel;
  Motor rightWheel;
  ColorSensor colorSensor;
  
  // 制御定数
  static const float Kp;        // 比例定数
  static const float Kd;        // 微分定数
  static const int bias;        // バイアス
  static const int target;      // 目標値
  
  // 適応的速度制御用定数
  static const int DEFAULT_BASE_SPEED = 50;    // デフォルト基本速度
  static const int SLOW_BASE_SPEED = 30;       // 青色検知後の低速
  static const int MIN_SPEED = 25;             // 最低速度
  
  // PID制御用
  mutable int mPreviousError;   // 前回のエラー値（D制御用）
  bool mIsInitialized;          // 初期化フラグ
  bool mLineTraceEnabled;       // ライントレース有効フラグ
  bool mBlueDetectionEnabled;   // 青色検知有効フラグ
  int mBlueDetectionCount;      // 青色検知回数カウンタ
  int mCurrentBaseSpeed;        // 現在の基本速度
  bool mIsStopped;              // 完全停止フラグ
  
  // 初期処理用フラグ
  bool mInitialSequenceCompleted;       // 初期処理完了フラグ
  unsigned long mInitialStartTime;      // 初期処理開始時刻
  
  // 青色検知用定数
  static const int BLUE_THRESHOLD;      // 青色判定閾値
  static const int COLOR_DIFF_THRESHOLD; // 他色との差の閾値
  
  // 前進制御用定数
  static const float WHEEL_DIAMETER_CM;  // ホイール直径 (cm)
  
  // 曲がり方向の定義
  enum class TurnDirection {
    STRAIGHT,  // 直進
    LEFT,      // 左曲がり
    RIGHT      // 右曲がり
  };
  
  // メソッド
  float calcPropValue(int diffReflection);    // PD制御値計算
  int calcAdaptiveSpeed(float turn);          // 適応的速度計算
  int calDiffReflection() const;              // 反射光差分計算
  bool detectBlue() const;                    // 青色検知メソッド
  void moveForward(float distanceCm, TurnDirection direction = TurnDirection::STRAIGHT, float turnIntensity = 0.5f);  // 前進＋曲がりメソッド
  void setLineTraceEnabled(bool enabled);     // ライントレース有効/無効設定
  bool isLineTraceEnabled() const;            // ライントレース状態取得
  void setBlueDetectionEnabled(bool enabled); // 青色検知有効/無効設定
  bool isBlueDetectionEnabled() const;        // 青色検知状態取得
  void executeBlueAction();                   // 青色検知時の動作実行
  void waitForStabilization();                // 動作安定化待機
  void setSlowMode(bool enabled);             // 低速モード設定
  int getCurrentBaseSpeed() const;            // 現在の基本速度取得
  void setCompleteStop(bool stopped);         // 完全停止設定
  bool isStopped() const;                     // 停止状態取得
  
  // 初期処理関連メソッド
  void performInitialSequence();             // 初期処理実行
  bool detectBlack() const;                   // 黒色検知メソッド
};
