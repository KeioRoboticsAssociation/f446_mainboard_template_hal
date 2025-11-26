#include "Encoder.hpp"
#include <cmath>
#include <math.h>

// エンコーダ（TIMエンコーダモード）を初期化
Encoder::Encoder(TIM_HandleTypeDef* htim, bool invert, float gear_ratio, uint8_t device_id)
: htim_(htim),
  invert_(invert),
  arr_(0),
  is_32bit_(false),
  pos_counts_(0),
  prev_cnt_(0),
  vel_cps_(0.0f),
  vel_cps_filt_(0.0f),
  prev_ms_(0),
  cpr_(1) // 0除算回避
{
  gear_ratio_ = (gear_ratio > 0.0f ? gear_ratio : 1.0f);
  device_id_ = device_id;
  // 事前係数を初期化
  recomputeFactors();
}

// DWT(CYCCNT) を有効化（高分解能タイマ利用）
static inline void dwt_enable_local() {
  // DWT(CYCCNT) を有効化（既に有効ならそのまま）
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

// カウンタ開始と内部状態の初期化
void Encoder::start() {
  // タイマ仕様を取得
  arr_ = __HAL_TIM_GET_AUTORELOAD(htim_);
  if (arr_ == 0) {
    // まだ初期化されていない可能性。最低限1にして動作継続
    arr_ = 0xFFFFu;
  }

  // 32bit判定：ARRが32bit上限近ければ32bitとみなす（TIM2/TIM5等）
  // 実用上は Instance==TIM2 || TIM5 で判定してもOK
  is_32bit_ = (arr_ > 0xFFFFu);

  // カウンタ開始（CubeMXがEncoderMode設定している前提）
  HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);

  // DWTを有効化し、高分解能タイムベースを準備
  dwt_enable_local();

  prev_cnt_ = readCNT();
  prev_ms_  = HAL_GetTick(); // 互換目的（未使用）
  prev_cyc_ = DWT->CYCCNT;
  vel_cps_  = 0.0f;
}

void Encoder::enableVirtualTracking(const float* velocity_rad_s_ptr, float initial_pos_rad) {
  virtual_mode_enabled_ = true;
  virtual_velocity_ptr_ = velocity_rad_s_ptr;
  virtual_pos_rad_ = initial_pos_rad;
  virtual_vel_rad_s_ = 0.0f;
  // 速度LPFは使用しないため生値をそのまま保持
  vel_cps_ = 0.0f;
  vel_cps_filt_ = 0.0f;
  if (rad_per_count_ != 0.0f) {
    const float counts_f = virtual_pos_rad_ / rad_per_count_;
    pos_counts_ = static_cast<int64_t>(llroundf(counts_f));
  } else {
    pos_counts_ = 0;
  }
  prev_cyc_ = DWT->CYCCNT;
}

void Encoder::disableVirtualTracking() {
  virtual_mode_enabled_ = false;
  virtual_velocity_ptr_ = nullptr;
  virtual_vel_rad_s_ = 0.0f;
  // 実エンコーダ値と同期
  prev_cnt_ = readCNT();
  prev_cyc_ = DWT->CYCCNT;
}

void Encoder::setVirtualPositionRad(float pos_rad) {
  virtual_pos_rad_ = pos_rad;
  if (rad_per_count_ != 0.0f) {
    const float counts_f = virtual_pos_rad_ / rad_per_count_;
    pos_counts_ = static_cast<int64_t>(llroundf(counts_f));
  } else {
    pos_counts_ = 0;
  }
}

// 現在のタイマカウンタ値を取得
inline uint32_t Encoder::readCNT() const {
  // CNTは16/32bit。読み出しはアトミックでOK
  return __HAL_TIM_GET_COUNTER(htim_);
}

// オーバーフロー補正つきで累積カウントを更新
void Encoder::updateOverflowCompensation(uint32_t curr_cnt) {
  // Δを符号付きで計算（オーバーフロー/アンダーフロー補正）
  // 有効レンジは [0 .. arr_]。カウント幅は arr_+1
  const int64_t mod = static_cast<int64_t>(arr_) + 1;
  int64_t delta = static_cast<int64_t>(curr_cnt) - static_cast<int64_t>(prev_cnt_);

  // 最短距離に折り畳む（リング上の距離）
  if (delta >  (mod / 2))  delta -= mod;
  if (delta < -(mod / 2))  delta += mod;

  if (invert_) delta = -delta;

  pos_counts_ += delta;
  prev_cnt_ = curr_cnt;
}

// dt算出→Δカウントから速度を更新（LPF対応）
void Encoder::sample() {
  // 高分解能のサイクル差からdt[s]を算出（オーバーフローはuint32減算で自然に処理）
  const uint32_t now_cyc = DWT->CYCCNT;
  const uint32_t dt_cyc  = now_cyc - prev_cyc_;
  // 0クロックまたは極小dtはガード（約1us未満など）
  if (dt_cyc == 0) return;
  // 除算削減: dt_s = dt_cyc * inv(SystemCoreClock)。invは初回評価のみ。
  static float inv_sysclk = 0.0f;
  if (inv_sysclk == 0.0f) {
    inv_sysclk = 1.0f / static_cast<float>(SystemCoreClock);
  }
  const float dt_s = static_cast<float>(dt_cyc) * inv_sysclk;
  if (dt_s <= 0.0f) return;

  prev_cyc_ = now_cyc;

  if (virtual_mode_enabled_) {
    float vel = 0.0f;
    if (virtual_velocity_ptr_) {
      vel = *virtual_velocity_ptr_;
    }
    virtual_vel_rad_s_ = vel;
    virtual_pos_rad_ += vel * dt_s;
    if (rad_per_count_ != 0.0f) {
      const float counts_f = virtual_pos_rad_ / rad_per_count_;
      pos_counts_ = static_cast<int64_t>(llroundf(counts_f));
    }
    vel_cps_ = (rads_per_cps_ != 0.0f) ? (vel / rads_per_cps_) : 0.0f;
    vel_cps_filt_ = vel_cps_;
    return;
  }

  const uint32_t curr_cnt = readCNT();
  // 位置更新（オーバーフロー補正付き）
  const int64_t before = pos_counts_;
  updateOverflowCompensation(curr_cnt);
  const int64_t after = pos_counts_;

  const int64_t dcounts = after - before;
  // 速度 [counts/s]（生）: 除算はこの1回のみ
  const float vel_raw_cps = static_cast<float>(dcounts) / dt_s;

  // LPF（一次遅れ）。tau<=0 なら無効。
  if (vel_lpf_tau_s_ > 0.0f) {
    // 除算削減: k = dt_s * (1/tau)。inv_tauは設定時に前計算。
    float k = dt_s * inv_vel_lpf_tau_;
    if (k > 1.0f) k = 1.0f; // 過大な一気追従を抑制
    vel_cps_filt_ += k * (vel_raw_cps - vel_cps_filt_);
    vel_cps_ = vel_cps_filt_;
  } else {
    vel_cps_ = vel_raw_cps;
  }
  virtual_pos_rad_ = static_cast<float>(pos_counts_) * rad_per_count_;
  virtual_vel_rad_s_ = vel_cps_ * rads_per_cps_;
}

// 累積位置カウントを外部値に設定
void Encoder::setPositionCounts(int64_t counts) {
  pos_counts_ = counts;
  prev_cnt_   = readCNT(); // 現在の生CNTを基準にする
  syncVirtualFromCounts();
}

// 位置[rad]を返す（減速比/CPR考慮）
float Encoder::positionRad() const {
  // CPR未設定なら0
  if (cpr_ == 0) return 0.0f;
  if (virtual_mode_enabled_) {
    return virtual_pos_rad_;
  }
  // 除算削減: 事前係数で乗算
  return static_cast<float>(pos_counts_) * rad_per_count_;
}

// 位置[deg]を返す
float Encoder::positionDeg() const {
  if (cpr_ == 0) return 0.0f;
  return static_cast<float>(pos_counts_) * deg_per_count_;
}

// 速度[rev/s]を返す
float Encoder::velocityRps() const {
  if (cpr_ == 0) return 0.0f;
  // 除算削減: 事前係数で乗算
  return vel_cps_ * rps_per_cps_;
}

// 速度[deg/s]を返す
float Encoder::velocityDegs() const {
  return velocityRps() * 360.0f;
}

// 速度[rad/s]を返す
float Encoder::velocityRads() const {
  // 除算削減: 事前係数で乗算
  if (virtual_mode_enabled_) {
    return virtual_vel_rad_s_;
  }
  return vel_cps_ * rads_per_cps_;
}

// Z相検出時に原点復帰（累積位置をゼロ）
void Encoder::onIndexPulse() {
  // Zパルスで原点復帰（方針：累積位置=0）
  // 必要なら現在の生CNTも0にリロードしたいが、他影響を避け累積だけゼロに。
  setPositionCounts(0);
}

// 現在位置を指定角度[deg]にリセット
void Encoder::setNowAsDeg(float deg) {
  // CPR が 0 だと 0除算なので保険（このクラスは既定で1）
  if (cpr_ == 0) cpr_ = 1;
  const float rev = deg / 360.0f;
  const int64_t counts = static_cast<int64_t>(llroundf(rev * (static_cast<float>(cpr_) * (gear_ratio_ > 0.0f ? gear_ratio_ : 1.0f))));
  setPositionCounts(counts);            // 累積位置のみ更新、CNTは触らない
}

// 現在位置を[0,360)に正規化して設定
void Encoder::setNowAsDegWrap360(float deg) {
  if (cpr_ == 0) cpr_ = 1;
  float d = fmodf(deg, 360.0f);     // [-360, 360) に正規化
  if (d < 0.0f) d += 360.0f;            // [0, 360)
  const float rev = d / 360.0f;
  const int64_t counts = static_cast<int64_t>(llroundf(rev * (static_cast<float>(cpr_) * (gear_ratio_ > 0.0f ? gear_ratio_ : 1.0f))));
  setPositionCounts(counts);
}

// 現在位置を指定角度[rad]にリセット
void Encoder::setNowAsRad(float rad) {
  if (cpr_ == 0) cpr_ = 1;
  const float rev = rad / (2.0f * static_cast<float>(M_PI));
  const int64_t counts = static_cast<int64_t>(llroundf(rev * (static_cast<float>(cpr_) * (gear_ratio_ > 0.0f ? gear_ratio_ : 1.0f))));
  setPositionCounts(counts);
}

// 生のタイマカウンタ値を返す
uint32_t Encoder::rawCounter() const {
  // 内部ヘルパ（private）の readCNT() を使っても、直接 CNT を読んでもOK
  return readCNT();
  // もしくは：return __HAL_TIM_GET_COUNTER(htim_);
}

// 変換係数の再計算（CPR/ギア比更新時に呼ばれる）
void Encoder::recomputeFactors() {
  const float eff_cpr = static_cast<float>(cpr_) * (gear_ratio_ > 0.0f ? gear_ratio_ : 1.0f);
  inv_eff_cpr_ = (eff_cpr > 0.0f) ? (1.0f / eff_cpr) : 0.0f;
  // 角度変換係数
  rad_per_count_ = 2.0f * static_cast<float>(M_PI) * inv_eff_cpr_;
  deg_per_count_ = 360.0f * inv_eff_cpr_;
  // 速度変換係数
  rps_per_cps_   = inv_eff_cpr_;
  rads_per_cps_  = 2.0f * static_cast<float>(M_PI) * inv_eff_cpr_;
  // 速度LPF係数（1/τ）。τ<=0は無効
  inv_vel_lpf_tau_ = (vel_lpf_tau_s_ > 0.0f) ? (1.0f / vel_lpf_tau_s_) : 0.0f;
  syncVirtualFromCounts();
}

void Encoder::syncVirtualFromCounts() {
  if (!virtual_mode_enabled_) return;
  virtual_pos_rad_ = static_cast<float>(pos_counts_) * rad_per_count_;
}
