#include "PerfMonitor.hpp"

// DWT(CYCCNT) を有効化して計測可能化
void PerfMonitor::init() {
  dwt_enable();
}

// 周期が来ていれば利用率などの統計をPublish
void PerfMonitor::maybePublish(uint32_t now_ms) {
  if (frame_id_ == 0) return; // 出力無効
  if ((now_ms - last_pub_ms_) < publish_period_ms_) return;
  last_pub_ms_ = now_ms;

  // 平均周期[cycle]
  const double tim_period_cyc_avg = (tim_period_cnt_ ? (double)tim_period_sum_ / (double)tim_period_cnt_ : 0.0);
  const double pub_period_cyc_avg = (pub_period_cnt_ ? (double)pub_period_sum_ / (double)pub_period_cnt_ : 0.0);

  // 平均実行時間[cycle]
  const double tim_exec_cyc_avg = (tim_cyc_cnt_ ? (double)tim_cyc_sum_ / (double)tim_cyc_cnt_ : 0.0);

  // 利用率[%]
  const float tim_util_max_pct = (tim_period_cyc_avg > 0.0) ? (float)(100.0 * (double)tim_cyc_max_ / tim_period_cyc_avg) : 0.0f;
  const float tim_util_avg_pct = (tim_period_cyc_avg > 0.0) ? (float)(100.0 * tim_exec_cyc_avg / tim_period_cyc_avg)    : 0.0f;
  const float pub_util_max_pct = (pub_period_cyc_avg > 0.0) ? (float)(100.0 * (double)pub_cyc_max_ / pub_period_cyc_avg) : 0.0f;

  // 締切超過カウントを送信（従来のoverrunの代わり）
  pub_.publish(tim_util_max_pct, tim_util_avg_pct, pub_util_max_pct, tim_deadline_miss_);

  // ピークは区間ごとにクリア
  tim_cyc_max_ = 0;
  pub_cyc_max_ = 0;
  // 窓平均にしたい場合は以下も有効化
  // tim_cyc_sum_ = 0; tim_cyc_cnt_ = 0;
  // tim_period_sum_ = 0; tim_period_cnt_ = 0;
  // pub_period_sum_ = 0; pub_period_cnt_ = 0;
}
