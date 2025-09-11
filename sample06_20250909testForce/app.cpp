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
  pup_device_t *force_sensor = pup_force_sensor_get_device(force_sensor_port);
  while (!pup_force_sensor_touched(force_sensor)) {
    dly_tsk(10*1000);
  }
  printf("Sample06: ETrobo_TR Style Line Trace with Initial Sequence\n");
  
  tracer.init();
  sta_cyc(TRACER_CYC);
  
  // ライントレースを無限に実行（終了条件なし）
  while (1) {
    dly_tsk(100*1000); // 100msウェイト
    
    // 初期処理の状態をモニタリング（最初の数回のみ表示）
    static int monitorCount = 0;
    if (monitorCount < 50 && !tracer.isInitialSequenceCompleted()) {
      printf("初期処理実行中... (カウント: %d)\n", monitorCount);
      monitorCount++;
    } else if (monitorCount == 50) {
      printf("初期処理完了 - ライントレース開始\n");
      monitorCount++;
    }
  }
}
