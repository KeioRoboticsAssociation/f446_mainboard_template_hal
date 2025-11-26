#pragma once
#include "stm32f4xx_hal.h"
#include <cmath>
#include <cstdint>

/**
 * @brief Quadrature Encoder reader using STM32 TIM Encoder Mode.
 *
 * 要件:
 * - CubeMXでTIMxを Encoder Mode に設定（CH1/CH2ピンAF設定, Counter有効）
 * - MX_TIMx_Init() 後に start() を呼ぶ
 * - 速度は sample() を一定周期で呼んで更新（HAL_GetTick()利用）
 *
 * 仕様:
 * - 位置: 符号付き64bitの累積カウント (オーバーフロー補正)
 * - 速度: counts/sec（差分/Δt）
 * - CPR=Counts Per Revolution（4逓倍後のカウント/回転）を設定可能
 * - オプションのIndex(Z相)で原点復帰 (EXTI割込み等から onIndexPulse() を呼ぶ)
 */
class Encoder {
public:
  /**
   * @param htim   TIMハンドル（Encoder Modeで初期化済み）
   * @param invert trueなら方向反転
   */
  /**
   * @param htim       TIMハンドル（Encoder Modeで初期化済み）
   * @param invert     trueなら方向反転
   * @param gear_ratio 減速比（出力1回転あたりのモータ回転数）。例: 30:1 → 30.0f
   */
  explicit Encoder(TIM_HandleTypeDef *htim, bool invert = false, float gear_ratio = 1.0f, uint8_t device_id = 0);

  /** カウンタ開始（TIMエンコーダ＋更新割込みは不要） */
  void start();

  /** 位置・速度更新用。一定周期で呼ぶ（例: 1ms〜10ms） */
  void sample();

  // ==== 仮想エンコーダ機能 ====
  // forward軸など、実エンコーダが利用できない場合に速度指令を積分して
  // 自己位置/速度を生成する。enable後は sample() が指令速度を積分し、
  // position*/velocity* 系アクセサは生成した値を返す。
  void enableVirtualTracking(const float* velocity_rad_s_ptr, float initial_pos_rad);
  void enableVirtualTracking(const float* velocity_rad_s_ptr) {
    enableVirtualTracking(velocity_rad_s_ptr, positionRad());
  }
  void disableVirtualTracking();
  bool isVirtualTrackingEnabled() const { return virtual_mode_enabled_; }
  void setVirtualPositionRad(float pos_rad);

  // ---- 位置系 ----
  /** 現在の累積位置 [counts] を返す（64bit） */
  int64_t positionCounts() const { return pos_counts_; }

  /** 累積位置を上書き（ゼロリセット等） */
  void setPositionCounts(int64_t counts);

  /** 角度[rad]（CPRが設定済みの場合） */
  float positionRad() const;

  /** 角度[deg]（CPRが設定済みの場合） */
  float positionDeg() const;

  // ---- 速度系（sample()後に更新）----
  /** 速度 [counts/s] */
  float velocityCps() const { return vel_cps_; }

  /** 速度 [rev/s]（CPRが設定済みの場合） */
  float velocityRps() const;

  /** 速度 [deg/s]（CPRが設定済みの場合） */
  float velocityDegs() const;

  /** 速度 [rad/s]（CPRが設定済みの場合） */
  float velocityRads() const;

  // ---- CPR設定 ----
  /**
   * @brief Counts Per Revolution を設定（4逓倍後の総カウント）
   * 例) エンコーダ仕様: 500 PPR(=A相500立上り), 4逓倍なら CPR=2000
   */
  void setCPR(uint32_t cpr) { cpr_ = (cpr == 0 ? 1u : cpr); recomputeFactors(); }

  uint32_t getCPR() const { return cpr_; }

  // ---- ギア比設定 ----
  /**
   * @brief 減速機のギア比を設定（出力1回転あたりのエンコーダ回転数）。
   * 例) モータ軸にエンコーダがあり、減速比が 30:1 の場合は 30。
   * 有効CPR = CPR * gear_ratio として角度/速度変換に使用。
   */
  void setGearRatio(float r) { gear_ratio_ = (r <= 0.0f ? 1.0f : r); recomputeFactors(); }
  float getGearRatio() const { return gear_ratio_; }

  // ---- Index(Z相)対応（任意）----
  /** Z相パルス割り込み等から呼ぶと原点復帰（累積位置=0） */
  void onIndexPulse();

  /** 方向反転の切替 */
  void setInvert(bool inv) { invert_ = inv; }
  bool isInverted() const { return invert_; }

  // 速度LPF設定（一次遅れ）。tau_s <= 0 でフィルタ無効。
  void setVelocityLpfTau(float tau_s) {
    vel_lpf_tau_s_ = (tau_s <= 0.0f ? 0.0f : tau_s);
    // 除算削減のため 1/τ を保持
    inv_vel_lpf_tau_ = (vel_lpf_tau_s_ > 0.0f) ? (1.0f / vel_lpf_tau_s_) : 0.0f;
  }
  float velocityLpfTau() const { return vel_lpf_tau_s_; }
  
  // 角度補正：現在位置を指定角度[deg]だとみなす（正負や360超えをそのまま許容）
  void setNowAsDeg(float deg);

  // 角度補正（0..360にラップしてから設定したい場合）
  void setNowAsDegWrap360(float deg);

  // ラジアン版が必要なら
  void setNowAsRad(float rad);

  // タイマの生カウンタ（CNTレジスタ）をそのまま返す
  uint32_t rawCounter() const;

private:
  inline uint32_t readCNT() const;
  void updateOverflowCompensation(uint32_t curr_cnt);
  // 変換係数を事前計算（除算削減のため）。
  void recomputeFactors();
  void syncVirtualFromCounts();

private:
  TIM_HandleTypeDef *htim_;
  bool invert_;

  // タイマ仕様
  uint32_t arr_;  // Auto-Reload（最大カウント値）
  bool is_32bit_; // TIM2/TIM5等=32bit

  // 位置
  int64_t pos_counts_; // 累積位置（拡張）
  uint32_t prev_cnt_;  // 直近の生CNT

  // 速度
  float vel_cps_;    // counts/s
  float vel_cps_filt_; // filtered counts/s
  uint32_t prev_ms_; // 前回サンプル時刻 [ms]（互換目的: DWT移行後は未使用）
  uint32_t prev_cyc_ = 0; // 前回サンプル時刻 [cycle]（DWT CYCCNT）
  float vel_lpf_tau_s_ = 0.0f; // 速度LPF時定数[s]（0で無効）

  // 変換
  uint32_t cpr_; // Counts Per Rev (4逓倍後)
  float    gear_ratio_ = 1.0f; // 減速比（>1で減速）
  uint8_t  device_id_ = 0;
  // 事前計算済みの係数（CPU負荷削減）
  float inv_eff_cpr_ = 1.0f;     // 1 / (cpr_ * gear_ratio_)
  float rad_per_count_ = 0.0f;   // 2π * inv_eff_cpr_
  float deg_per_count_ = 0.0f;   // 360 * inv_eff_cpr_
  float rps_per_cps_ = 0.0f;     // inv_eff_cpr_
  float rads_per_cps_ = 0.0f;    // 2π * inv_eff_cpr_
  float inv_vel_lpf_tau_ = 0.0f; // 1/tau（tau<=0のとき0）

  // 仮想エンコーダ（速度積分）用の状態
  bool         virtual_mode_enabled_ = false;
  const float* virtual_velocity_ptr_ = nullptr; // 指令角速度[rad/s]
  float        virtual_pos_rad_ = 0.0f;
  float        virtual_vel_rad_s_ = 0.0f;
public:
  uint8_t deviceID() const { return device_id_; }
};
