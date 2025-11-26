#pragma once

#include <cmath>
#include <cstdint>
#include "DCMotor.hpp"
#include "Encoder.hpp"
#include "Msg.hpp"   // ROSToF446 / F446ToROS

/**
 * VelocityPIController（角速度PI制御）
 * - 角速度誤差[rad/s]に対してPIでモータDutyを制御。
 * - 無負荷回転数(6160rpm)に基づく簡易フィードフォワードを併用。
 *
 * 注意:
 * - DCMotor::setDuty(d) は d∈[-1,1] を前提。負方向のPWMが危険な実装では
 *   allow_reverse=false（既定）を維持し、負のDutyが出ないようにする。
 */
class VelocityPIController {
public:
  VelocityPIController(DCMotor& motor, Encoder& encoder);

  void setGains(float kp, float ki);
  void setTargetRadPerSec(float omega_rad_s);
  void setDutyLimit(float min_duty, float max_duty);
  void setAllowReverse(bool enable);
  void setTargetScale(float s) { target_scale_ = s; }
  // FFに乗算するゲイン（既定=1.0）。k_ff := k_ff * ff_gain
  void setFeedforwardGain(float g) { ff_gain_ = g; }
  // FF係数を直接上書き（単位: duty / (rad/s)。r軸のmm/s系にもそのまま適用可）
  void setFeedforwardScale(float k) { k_ff_scale_ = k; }

  // FF/バイアスの符号取り扱い
  // - FFの符号を入力座標の正負に対して反転するかを設定（既定=false=反転しない）
  void setFeedforwardInvert(bool invert) { ff_invert_ = invert; }
  // - 静的バイアスで "up" を入力座標の正方向に対応させるか（既定=false: up=負方向/down=正方向）
  void setBiasUpIsPositive(bool enable) { bias_up_is_positive_ = enable; }
  // 極性の自動判別/手動指定は廃止（物理で統一）

  // 重力・静摩擦補償用の静的デューティバイアス（方向別、符号可）
  // up_duty:   上向き（負方向）判定時にそのまま加える値（範囲[-1,1]）
  // down_duty: 下向き（正方向）判定時にそのまま加える値（範囲[-1,1]）
  // deadband: 符号決定用の速度しきい値[rad/s]
  void setStaticDutyBias(float up_duty, float down_duty, float deadband_rad_s = 0.2f) {
    // [-1,1] にクリップ（負値も許容）
    auto clip11 = [](float v){ return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); };
    bias_up_duty_ = clip11(up_duty);
    bias_down_duty_ = clip11(down_duty);
    bias_deadband_rad_s_ = (deadband_rad_s < 0.0f ? 0.0f : deadband_rad_s);
  }

  // 座標系ベース指定：正方向/負方向 でバイアスを指定（推奨API）
  // pos_duty: 座標の正方向に動くときに加えるバイアス
  // neg_duty: 座標の負方向に動くときに加えるバイアス
  // 備考: 実際の出力デューティへの写像では内部でモータ極性に追従するため、
  //       配線極性（インバート）を意識せずに数値を決められる。
  void setDutyBiasBySign(float pos_duty, float neg_duty, float deadband_rad_s = 0.2f) {
    auto clip11 = [](float v){ return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); };
    // 「up=正方向」に対応させるフラグを立て、値を割当
    bias_up_is_positive_ = true;
    bias_up_duty_ = clip11(pos_duty);
    bias_down_duty_ = clip11(neg_duty);
    bias_deadband_rad_s_ = (deadband_rad_s < 0.0f ? 0.0f : deadband_rad_s);
  }

  // 目標スムージング（一次遅れLPF, tau<=0で無効）
  void setTargetLpfTau(float tau_s) {
    target_lpf_tau_s_ = (tau_s <= 0.0f ? 0.0f : tau_s);
    // 除算削減: 1/τ を前計算
    inv_target_lpf_tau_ = (target_lpf_tau_s_ > 0.0f) ? (1.0f / target_lpf_tau_s_) : 0.0f;
  }

  // 目標の加速度リミット [rad/s^2]（<=0で無効）
  void setMaxAccel(float a_rad_s2) { max_acc_rad_s2_ = (a_rad_s2 <= 0.0f ? 0.0f : a_rad_s2); }

  // コマンドウォッチドッグ（ms）。timeout_ms<=0で無効。fallback_targetはタイムアウト時目標。
  void setCommandTimeout(uint32_t timeout_ms, float fallback_target_rad_s = 0.0f) {
    cmd_timeout_ms_ = timeout_ms; cmd_fallback_rad_s_ = fallback_target_rad_s;
  }

  // 最終コマンド受信時刻の参照元（HAL_GetTick()と同一タイムベース）
  void setCommandTimestampSource(const volatile uint32_t* last_cmd_ms_ptr) { last_cmd_ms_ptr_ = last_cmd_ms_ptr; }

  // 制御ステップ[秒]で周期呼び出し
  void update(float dt_s);

  // ROSブリッジ連携（main.cppを薄く保つための接点）
  void setCommandSource(const ROSToF446* cmd) { cmd_src_ = cmd; }
  void setStatusSink(F446ToROS* st) { status_sink_ = st; }
  // 外部参照をバインド（特定フィールドに入出力を切替）
  // - target_ptr が設定されている場合はそれを優先
  // - pos_ptr/vel_ptr が設定されている場合はそこへ書き出し
  void bindExternalTarget(const float* target_ptr) { ext_target_ptr_ = target_ptr; }
  void bindExternalStatus(float* pos_ptr, float* vel_ptr) {
    ext_pos_ptr_ = pos_ptr; ext_vel_ptr_ = vel_ptr;
  }
  // ステータスに書き出す位置を [-pi, pi] にラップ（角度軸用）。既定は無効。
  void setWrapPositionToPi(bool enable) { wrap_pos_pi_ = enable; }
  // 角度依存の重力FF: duty = amp * cos(theta + phase)。invert=trueで符号反転。
  void setGravityCosFF(float amp_duty, float phase_rad = 0.0f, bool invert = false) {
    if (amp_duty < 0.0f) amp_duty = -amp_duty; // 振幅は常に非負
    grav_ff_amp_ = amp_duty;
    grav_ff_phase_ = phase_rad;
    grav_ff_invert_ = invert;
    grav_ff_auto_polarity_ = false; // 明示指定時は自動反転を無効化
    grav_ff_enable_ = (grav_ff_amp_ > 0.0f);
  }
  // モータとエンコーダの極性から自動で符号を決める重力FF
  // 有効時、毎ステップで motor_.polaritySign() と enc_.isInverted() を見て符号を決定。
  void setGravityCosFFAutoPolarity(float amp_duty, float phase_rad = 0.0f) {
    if (amp_duty < 0.0f) amp_duty = -amp_duty;
    grav_ff_amp_ = amp_duty;
    grav_ff_phase_ = phase_rad;
    grav_ff_auto_polarity_ = true;
    grav_ff_enable_ = (grav_ff_amp_ > 0.0f);
  }
  // To be called from a fixed-rate timer ISR; handles sample, dt, update
  void onTimerTick();

  // アクセサ
  float duty() const { return duty_; }
  float target() const { return target_rad_s_; }
  float measured() const { return measured_rad_s_; }
  float feedforwardDutyFor(float target_rad_s) const;

  void reset();
  // 開ループでデューティを直接出力（エンコーダ参照を一時停止したいケース向け）
  void setOpenLoopDuty(float duty);

private:
  // 定数
  static constexpr float kNoLoadRpm   = 6160.0f;
  static constexpr float kNoLoadRad_s = kNoLoadRpm / 60.0f * 2.0f * static_cast<float>(M_PI);

  // 参照先
  DCMotor& motor_;
  Encoder& enc_;

  // 状態変数
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float integ_ = 0.0f;      // integrator state
  float target_rad_s_ = 0.0f;
  float measured_rad_s_ = 0.0f;
  float duty_ = 0.0f;       // last duty command
  float duty_min_ = 0.0f;
  float duty_max_ = 1.0f;
  bool  allow_reverse_ = false;
  float target_scale_ = 1.0f;   // external target scaling (e.g., motor->output axis)

  // 目標成形
  float target_lpf_tau_s_ = 0.0f;     // LPF tau [s] (0=disabled)
  float max_acc_rad_s2_   = 0.0f;     // slew limit [rad/s^2] (0=disabled)
  float target_filtered_rad_s_ = 0.0f; // internal filtered target state
  float inv_target_lpf_tau_ = 0.0f;    // 1/τ for LPF (0 if disabled)

  // 静的デューティバイアス（重力・静摩擦補償）
  float bias_up_duty_ = 0.0f;      // 上向き（負方向）に使う|duty|
  float bias_down_duty_ = 0.0f;    // 下向き（正方向）に使う|duty|
  float bias_deadband_rad_s_ = 0.2f; // 符号決定のデッドバンド
  bool  bias_up_is_positive_ = false; // true: up=正方向, false: up=負方向（後方互換）

  // コマンド鮮度ウォッチドッグ
  uint32_t cmd_timeout_ms_ = 0;            // 0=disabled
  float    cmd_fallback_rad_s_ = 0.0f;     // fallback target on timeout
  const volatile uint32_t* last_cmd_ms_ptr_ = nullptr; // source of last command tick

  // 任意の連携（外部参照の入出力先）
  const ROSToF446* cmd_src_ = nullptr;  // read target from here if set
  F446ToROS*       status_sink_ = nullptr; // write back simple status if set
  const float*     ext_target_ptr_ = nullptr;  // optional external target src
  float*           ext_pos_ptr_ = nullptr;     // optional external pos sink
  float*           ext_vel_ptr_ = nullptr;     // optional external vel sink

  // 事前計算したFFスケール: gear / kNoLoadRad_s
  float k_ff_scale_ = 0.0f;
  float ff_gain_ = 1.0f; // 乗算ゲイン（軸毎の補正）
  bool  ff_invert_ = false;
  bool  wrap_pos_pi_ = false; // ステータス出力の角度ラップ有無（[-pi,pi]）
  // 角度依存の重力補償（cosベース）
  bool  grav_ff_enable_ = false;
  float grav_ff_amp_ = 0.0f;     // dutyスケール（cos=±1での|duty|）
  float grav_ff_phase_ = 0.0f;   // 物理ゼロ調整 [rad]
  bool  grav_ff_invert_ = false; // 極性反転（座標系→dutyの不一致対策）
  bool  grav_ff_auto_polarity_ = false; // モータ/エンコーダ極性から自動反転

  // タイミング（インスタンス毎のDWTベースdt）
  uint32_t prev_cyc_ = 0;
  bool     dwt_init_ = false;

  // ヘルパ
  static inline float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }
  static inline float wrap_pi(float a) {
    while (a >  static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
  }
};
