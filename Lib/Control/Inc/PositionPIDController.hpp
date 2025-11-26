#pragma once

#include <cstdint>
#include <cmath>
#include "Encoder.hpp"
#include "VelocityPIController.hpp"

// PositionPIDController（位置→速度の外側PID）
// - 位置誤差から目標速度を生成する外側ループPID。
// - ROSのpid_controllerと同等の動きを意識：
//   * e = target_pos - measured_pos（thetaは[-pi, pi]にラップ可能）
//   * reset_threshold: |e| < 閾値 で積分リセット＆出力0
//   * 出力の飽和は ±max_vel（内側ループの単位系に合わせる）
//   * dtはDWT(CYCCNT)で算出
// - 直線軸は既存の単位系（ギア比換算でmm相当をrad風に扱う）を踏襲
class PositionPIDController {
public:
  PositionPIDController(Encoder& enc)
  : enc_(enc) {}

  void setGains(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd; reset();
  }

  void setResetThreshold(float thr) { reset_threshold_ = (thr < 0.0f ? 0.0f : thr); }
  void setMaxVelocity(float vmax) { max_vel_ = (vmax < 0.0f ? 0.0f : vmax); }
  // 角度加速度制限: 位置制御の出力速度uの変化量を |du| <= max_accel_*dt に制限（rad/s^2）
  void setMaxAcceleration(float amax) { max_accel_ = (amax < 0.0f ? 0.0f : amax); }
  void setThetaWrap(bool enable) { theta_wrap_ = enable; }

  // ノーゴー機能は廃止。
  // 状態付き経路選択（CW/CCWロック）。ノーゴーなしで向きを固定して安定化。
  void setStatefulDirectionEnabled(bool enable) { stateful_dir_enable_ = enable; }
  // select_epsは廃止。最短経路/ロック強制のみで扱う。
  void setStatefulDirectionParams(float /*select_eps_unused*/, float settle_err, float tgt_jump_thr) {
    settle_err_ = (settle_err < 0.0f ? 0.0f : settle_err);
    tgt_jump_thr_ = (tgt_jump_thr < 0.0f ? 0.0f : tgt_jump_thr);
  }
  void resetDirectionLock() { dir_lock_ = 0; }
  bool isDirectionLocked() const { return stateful_dir_enable_ && (dir_lock_ != 0); }
  // ノーゴー機能は廃止。

  // 位置リミット: 目標位置を[min_pos, max_pos]にクランプ。
  // min_pos >= max_pos の場合は無効化（無制限）。
  void setPositionLimits(float min_pos, float max_pos) {
    if (min_pos >= max_pos) {
      pos_limits_enable_ = false;
    } else {
      pos_limits_enable_ = true;
      pos_min_pos_ = min_pos;
      pos_max_pos_ = max_pos;
    }
  }
  // 位置リミットを無効化（無制限）
  void setPositionUnlimited() { pos_limits_enable_ = false; }

  // 外部I/Oをバインド
  void bindTargetPosition(const float* target_pos_ptr) { tgt_pos_ptr_ = target_pos_ptr; }
  void bindVelocityOutput(float* vel_out_ptr) { vel_out_ptr_ = vel_out_ptr; }

  // ウォッチドッグ: timeout_ms間コマンドが来なければ出力0＆リセット
  void setCommandTimeout(uint32_t timeout_ms) { cmd_timeout_ms_ = timeout_ms; }
  void setCommandTimestampSource(const volatile uint32_t* last_cmd_ms_ptr) { last_cmd_ms_ptr_ = last_cmd_ms_ptr; }

  // 固定周期（200Hz想定）で呼び出し
  void onTimerTick();

  float lastVelocityCmd() const { return last_vel_cmd_; }
  float lastTargetPosition() const { return last_target_pos_; }
  float lastMeasuredPosition() const { return last_meas_pos_; }
  bool withinResetZone() const { return in_reset_zone_; }
  void reset();

  // デバッグ用途: Kpのみをスケール（例: div>0 のとき 1/div 倍にするなど）
  void scaleKpBy(float scale) { kp_ *= scale; }

  // デバッグ用途: 最大速度MaxVelocityもスケール（div>0 のとき 1/div 倍など）
  void scaleMaxVelBy(float scale) {
    if (scale <= 0.0f) return;
    if (max_vel_ < 0.0f) max_vel_ = 0.0f;
    max_vel_ *= scale;
  }

private:
  static inline float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  static inline float wrap_pi(float a) {
    while (a >  M_PI) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -M_PI) a += 2.0f * static_cast<float>(M_PI);
    return a;
  }

  Encoder& enc_;

  // PID params/state
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float integ_ = 0.0f;
  float prev_e_ = 0.0f;

  // Behavior
  float reset_threshold_ = 0.0f; // |e|<thr => reset & output 0
  float max_vel_ = 0.0f;         // saturation (units must match axis conventions)
  float max_accel_ = 0.0f;       // slew on velocity command [rad/s^2] (0=disabled)
  bool  theta_wrap_ = false;     // wrap error to [-pi,pi]

  // 状態付き経路選択（ノーゴー無しのCW/CCWロック）
  bool  stateful_dir_enable_ = false;
  int   dir_lock_ = 0;          // +1:CCW, -1:CW, 0:未選択
  float last_tgt_norm_ = 0.0f;  // [0,2π) 正規化した直近ターゲット
  float settle_err_ = 0.02f;    // 到達判定[rad]
  float tgt_jump_thr_ = 0.3f;   // 再選択用の目標ジャンプ閾値[rad]

  // Position limit settings (linear axes想定。theta_wrap併用時は未使用想定)
  bool  pos_limits_enable_ = false;
  float pos_min_pos_ = 0.0f;
  float pos_max_pos_ = 0.0f;

  // IO bindings
  const float* tgt_pos_ptr_ = nullptr; // target position (rad or mm based on axis convention)
  float*       vel_out_ptr_ = nullptr; // velocity command (same unit family as encoder.velocity*)

  // Watchdog
  uint32_t cmd_timeout_ms_ = 200; // default 200ms
  const volatile uint32_t* last_cmd_ms_ptr_ = nullptr;

  // Timing
  uint32_t prev_cyc_ = 0;
  bool dwt_init_ = false;

  // Output cache
  float last_vel_cmd_ = 0.0f;
  float last_target_pos_ = 0.0f;
  float last_meas_pos_ = 0.0f;
  bool in_reset_zone_ = false;

private:
  static inline float norm_2pi(float a) {
    float t = fmodf(a, 2.0f * static_cast<float>(M_PI));
    if (t < 0.0f) t += 2.0f * static_cast<float>(M_PI);
    return t;
  }
  static inline float d_ccw(float from, float to) {
    // CCWで from→to の角距離 [0,2π)
    float d = to - from;
    while (d < 0.0f) d += 2.0f * static_cast<float>(M_PI);
    while (d >= 2.0f * static_cast<float>(M_PI)) d -= 2.0f * static_cast<float>(M_PI);
    return d;
  }
  float computeThetaErrorStateful(float pos_rad, float tgt_rad);
};
