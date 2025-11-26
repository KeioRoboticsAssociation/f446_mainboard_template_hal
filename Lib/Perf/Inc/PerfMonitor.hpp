#pragma once

#include <cstdint>
#include "stm32f4xx_hal.h"
#include "UartLink.hpp"

// 軽量パフォーマンス計測クラス
// - DWTで処理時間(サイクル)と周期(サイクル)を測定
// - 利用率[%]に変換してUartLinkで定期送信（frame_idは任意指定）
class PerfMonitor {
public:
  // 計測結果を送るフレームIDを指定（0で送信無効）。period_msは送信間隔。
  PerfMonitor(UartLink& link, uint8_t frame_id, uint32_t period_ms = 200)
  : link_(link), frame_id_(frame_id), publish_period_ms_(period_ms), pub_(link_, frame_id_) {}

  void init();

  // TIM周期処理の入口・出口で呼ぶ
  inline void onTimTickStart();
  inline void onTimTickEnd();

  // publish処理の入口・出口で呼ぶ
  inline void onPublishStart();
  inline void onPublishEnd();

  // 時刻(ms)を渡して、必要なら送信
  void maybePublish(uint32_t now_ms);

private:
  // DWT helpers
  static inline void dwt_enable() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  // 内部集計（TIM5用）
  // 実行時間
  volatile uint32_t tim_cyc_max_ = 0;
  volatile uint64_t tim_cyc_sum_ = 0;
  volatile uint32_t tim_cyc_cnt_ = 0;
  // 締切超過カウント（直近の実行時間 > 直近の周期）
  volatile uint32_t tim_deadline_miss_ = 0;
  // 前回IRQ開始サイクル、および直近周期[cycle]
  volatile uint32_t tim_last_irq_cyc_ = 0;
  volatile uint32_t tim_last_period_cyc_ = 0;
  // 周期統計
  volatile uint64_t tim_period_sum_ = 0;
  volatile uint32_t tim_period_cnt_ = 0;
  // 実行中フラグ（再入検知は行わない）
  volatile uint8_t  tim_running_ = 0;

  // publish用
  volatile uint32_t pub_cyc_max_ = 0;
  volatile uint32_t pub_last_start_cyc_ = 0;
  volatile uint64_t pub_period_sum_ = 0;
  volatile uint32_t pub_period_cnt_ = 0;

  // 送信間隔
  uint32_t publish_period_ms_ = 200;
  uint32_t last_pub_ms_ = 0;

  // 出力
  UartLink& link_;
  uint8_t frame_id_ = 0;
  UartLinkPublisher<float, float, float, uint32_t> pub_;
};

// ---- inline実装 ----
inline void PerfMonitor::onTimTickStart() {
  // 開始時刻を取得し、前回開始からの周期を更新
  tim_running_ = 1;
  const uint32_t start = DWT->CYCCNT;
  if (tim_last_irq_cyc_ != 0) {
    const uint32_t dt = start - tim_last_irq_cyc_;
    tim_last_period_cyc_ = dt;
    tim_period_sum_ += dt;
    tim_period_cnt_++;
  }
  tim_last_irq_cyc_ = start;
}

inline void PerfMonitor::onTimTickEnd() {
  const uint32_t cyc = DWT->CYCCNT - tim_last_irq_cyc_;
  if (cyc > tim_cyc_max_) tim_cyc_max_ = cyc;
  tim_cyc_sum_ += cyc;
  tim_cyc_cnt_++;
  // 締切超過（実行が周期を超えた）を検出
  if (tim_last_period_cyc_ != 0 && cyc > tim_last_period_cyc_) {
    tim_deadline_miss_++;
  }
  tim_running_ = 0;
}

inline void PerfMonitor::onPublishStart() {
  const uint32_t start = DWT->CYCCNT;
  if (pub_last_start_cyc_ != 0) {
    const uint32_t pdt = start - pub_last_start_cyc_;
    pub_period_sum_ += pdt;
    pub_period_cnt_++;
  }
  pub_last_start_cyc_ = start;
}

inline void PerfMonitor::onPublishEnd() {
  const uint32_t cyc = DWT->CYCCNT - pub_last_start_cyc_;
  if (cyc > pub_cyc_max_) pub_cyc_max_ = cyc;
}
