#include "app.h"
#include <stdio.h>

#include "Tracer.h"
#include "spike/pup/forcesensor.h"

Tracer tracer;

using namespace spikeapi;

void tracer_task(intptr_t exinf) {
  tracer.run();
  ext_tsk();
}

void main_task(intptr_t unused) {
  printf("+---------------------------------+\n");
  printf("|   Press force sensor to start   |\n");
  printf("+---------------------------------+\n");
  /* フォースセンサーが押下されるまで待機 */
  pup_device_t *force_sensor = pup_force_sensor_get_device(PBIO_PORT_ID_D);
  while (!pup_force_sensor_touched(force_sensor)) {
    dly_tsk(10*1000);
  }
  printf("Sample06: ETrobo_TR Style Line Trace with Initial Sequence\n");
  
  // 初期処理付きでtracerを再初期化
  tracer.init();
  sta_cyc(TRACER_CYC);
  
  // 初期処理が未完了の場合のモニタリング
  int monitorCount = 0;
  while (!tracer.isInitialSequenceCompleted()) {
    dly_tsk(150*1000); // 100msウェイト
    if (monitorCount < 50) {
      printf("初期処理実行中... (カウント: %d)\n", monitorCount);
      monitorCount++;
    }
  }

  printf("初期処理完了 - ライントレース開始\n");

  // ライントレースを無限に実行（終了条件なし）
  while (1) {
    dly_tsk(100*1000); // 100msウェイト
  }
}