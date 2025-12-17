#include "VelocityPIController.hpp"
#include <cstdio>
#include <math.h>

// モータ/エンコーダ参照を受け取る速度PI制御器
VelocityPIController::VelocityPIController(DCMotor& motor, Encoder& encoder)
  : motor_(motor), enc_(encoder) {
  // Defaults
  setGains(0.1f, 5.0f);   // conservative defaults; tune as needed
  setDutyLimit(0.0f, 1.0f);
  setAllowReverse(false);
  target_filtered_rad_s_ = 0.0f;
  // 事前係数: k_ff_scale = gear / kNoLoadRad_s
  k_ff_scale_ = motor_.gearRatio() / kNoLoadRad_s;
}

// 比例/積分ゲインを設定
void VelocityPIController::setGains(float kp, float ki) {
  kp_ = kp;
  ki_ = ki;
}

// 目標角速度[rad/s]を設定
void VelocityPIController::setTargetRadPerSec(float omega_rad_s) {
  target_rad_s_ = omega_rad_s;
}

// デューティ下限/上限を設定
void VelocityPIController::setDutyLimit(float min_duty, float max_duty) {
  duty_min_ = fminf(min_duty, max_duty);
  duty_max_ = fmaxf(min_duty, max_duty);
}

// 逆転許可の有無を設定
void VelocityPIController::setAllowReverse(bool enable) { allow_reverse_ = enable; }

// 積分/デューティ状態をリセット
void VelocityPIController::reset() {
  integ_ = 0.0f;
  duty_  = 0.0f;
}

void VelocityPIController::setOpenLoopDuty(float duty) {
  // 既定のPIループを経由せずPWMを指示する暫定運用。上限等は従来設定を再利用。
  float u = clamp(duty, -1.0f, 1.0f);
  if (u >  duty_max_) u =  duty_max_;
  if (u < -duty_max_) u = -duty_max_;
  if (!allow_reverse_ && u < 0.0f) {
    u = 0.0f;
  }
  if (duty_min_ > 0.0f && u != 0.0f) {
    const float mag = fabsf(u);
    if (mag < duty_min_) {
      u = (u > 0.0f) ? duty_min_ : -duty_min_;
    }
  }
  duty_ = u;
  motor_.setDuty(duty_);
}

float VelocityPIController::feedforwardDutyFor(float target_rad_s) const {
  float tgt = target_rad_s;
  float meas = enc_.velocityRads();
  if (!allow_reverse_) {
    tgt = fabsf(tgt);
    meas = fabsf(meas);
  }

  const float u_ff = (k_ff_scale_ * ff_gain_) * (ff_invert_ ? -tgt : tgt);

  float u_grav = 0.0f;
  if (grav_ff_enable_) {
    const float theta = enc_.positionRad();
    const float c = cosf(theta + grav_ff_phase_);
    int sign = grav_ff_invert_ ? -1 : +1;
    if (grav_ff_auto_polarity_) {
      sign *= motor_.polaritySign();
      if (enc_.isInverted()) sign *= -1;
    }
    u_grav = static_cast<float>(sign) * (grav_ff_amp_ * c);
  }

  float u_bias = 0.0f;
  const float abs_tgt  = fabsf(tgt);
  const float abs_meas = fabsf(meas);
  int dir = 0;
  if (abs_tgt > bias_deadband_rad_s_) {
    dir = (tgt > 0.0f) ? +1 : -1;
  } else if (abs_meas > bias_deadband_rad_s_) {
    dir = (meas > 0.0f) ? +1 : -1;
  }
  if (dir != 0) {
    if (bias_up_is_positive_) {
      u_bias = (dir > 0) ? (+bias_up_duty_) : (+bias_down_duty_);
    } else {
      u_bias = (dir > 0) ? (+bias_down_duty_) : (+bias_up_duty_);
    }
  }

  float u = u_ff + u_grav + u_bias;
  if (u >  duty_max_) u =  duty_max_;
  if (u < -duty_max_) u = -duty_max_;

  if (duty_min_ > 0.0f) {
    const float mag = fabsf(u);
    if (mag > 0.0f && mag < duty_min_) {
      if (tgt != 0.0f) {
        u = (u >= 0.0f) ? duty_min_ : -duty_min_;
      } else {
        u = 0.0f;
      }
    }
  }

  if (!allow_reverse_ && u < 0.0f) {
    u = 0.0f;
  }

  return clamp(u, -1.0f, 1.0f);
}

  // 1ステップ更新（符号付きFF+PI, 対称飽和/アンチワインドアップ）
void VelocityPIController::update(float dt_s) {
  if (dt_s <= 0.0f) return;

  // Read measured speed [rad/s]
  measured_rad_s_ = enc_.velocityRads();

  // Reverse可否に応じて目標/実測の符号の扱いを決定
  float tgt  = target_rad_s_;
  float meas = measured_rad_s_;
  if (!allow_reverse_) {
    // 逆転不可: 絶対値制御（負目標は正に丸める）
    tgt  = fabsf(tgt);
    meas = fabsf(meas);
  }

  // Error [rad/s]（座標系）
  const float e = tgt - meas;

  // Feedforward（符号付き）: 出力軸目標[rad/s]→Dutyへの比例変換
  // kNoLoadRad_s はモータ軸の無負荷角速度。減速比を用いて換算。
  // 物理で極性を統一している前提。必要ならFFのみ反転を許可。
  const float u_ff = (k_ff_scale_ * ff_gain_) * (ff_invert_ ? -tgt : tgt);

  // 角度依存の重力FF（rapid等）。座標系のθに対して cos(θ+phase) に比例。
  // 仕様: θ=0 → +amp, θ=π/2 → 0, θ=π → -amp。
  float u_grav = 0.0f;
  if (grav_ff_enable_) {
    const float theta = enc_.positionRad();
    const float c = cosf(theta + grav_ff_phase_);
    int sign = grav_ff_invert_ ? -1 : +1;
    if (grav_ff_auto_polarity_) {
      // motor正方向と座標正の相関（+1/-1）と、エンコーダの反転を考慮
      sign *= motor_.polaritySign();
      if (enc_.isInverted()) sign *= -1;
    }
    u_grav = static_cast<float>(sign) * (grav_ff_amp_ * c);
  }

  // 静的デューティバイアス（重力/静摩擦補償）。
  // 符号判定はデッドバンド外では目標、デッドバンド内では実測で補助。
  float u_bias = 0.0f;
  const float abs_tgt  = fabsf(tgt);
  const float abs_meas = fabsf(meas);
  // 符号判定は「入力座標の正負」に対して行う（FF反転指定は考慮しない）
  int dir = 0; // +1: 正方向, -1: 負方向, 0: 無効
  if (abs_tgt > bias_deadband_rad_s_) {
    dir = (tgt > 0.0f) ? +1 : -1;
  } else if (abs_meas > bias_deadband_rad_s_) {
    dir = (meas > 0.0f) ? +1 : -1;
  }
  if (dir != 0) {
    if (bias_up_is_positive_) {
      // up を 正方向、down を 負方向 に対応させる（座標系ベース）
      u_bias = (dir > 0) ? (+bias_up_duty_) : (+bias_down_duty_);
    } else {
      // 従来の対応: up=負方向、down=正方向
      u_bias = (dir > 0) ? (+bias_down_duty_) : (+bias_up_duty_);
    }
    // 物理で極性を統一しているため、そのままduty系に加算
  }

  // PI terms（座標系の誤差をそのまま使用）
  const float p = kp_ * e;

  // Anti-windup: 符号付きuの飽和方向でのみ積分を停止
  float u_no_i = u_ff + u_grav + u_bias + p + ki_ * integ_; // 符号付きの仮出力
  bool at_hi = (u_no_i >=  duty_max_ - 1e-6f);
  bool at_lo = (u_no_i <= -duty_max_ + 1e-6f);
  if (!(at_hi && e > 0.0f) && !(at_lo && e < 0.0f)) {
    // 飽和方向に更に押し込むとき以外は積分
    integ_ += e * dt_s;
  }

  // 符号付き出力を算出し対称クランプ
  float u = u_ff + u_grav + u_bias + p + ki_ * integ_;
  if (u >  duty_max_) u =  duty_max_;
  if (u < -duty_max_) u = -duty_max_;

  // デッドゾーン/最低デューティの処理
  if (duty_min_ > 0.0f) {
    const float mag = fabsf(u);
    if (mag > 0.0f && mag < duty_min_) {
      if (tgt != 0.0f) {
        u = (u >= 0.0f) ? duty_min_ : -duty_min_;
      } else {
        u = 0.0f; // 目標0のときは出力ゼロへ
      }
    }
  }

  // 逆転不可なら負側をゼロにクリップ
  if (!allow_reverse_ && u < 0.0f) {
    u = 0.0f;
  }

  duty_ = u;

  motor_.setDuty(duty_);
  // printf("Duty: %f\n", duty_);
}

// 固定レートタイマから呼ばれ、dt算出・目標成形・updateを実施
void VelocityPIController::onTimerTick() {
  // エンコーダのサンプリングは外部（TIM5）で実施し、ここでは読み取りのみ。
  // dt はDWT(CYCCNT)でμs分解能算出（インスタンス毎に独立管理）
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    prev_cyc_ = DWT->CYCCNT;
    dwt_init_ = true;
  }
  if (!dwt_init_) {
    prev_cyc_ = DWT->CYCCNT;
    dwt_init_ = true;
  }
  const uint32_t now_cyc = DWT->CYCCNT;
  const uint32_t dt_cyc  = now_cyc - prev_cyc_;
  prev_cyc_ = now_cyc;
  // 除算削減: dt_s = dt_cyc * inv(SystemCoreClock)
  static float inv_sysclk = 0.0f;
  if (inv_sysclk == 0.0f) inv_sysclk = 1.0f / static_cast<float>(SystemCoreClock);
  const float dt_s = (dt_cyc == 0) ? 0.0f : (static_cast<float>(dt_cyc) * inv_sysclk);
  const uint32_t now_ms = HAL_GetTick();

  // 目標の取得優先度: ext_target_ptr_ -> 0
  float target_in = 0.0f;
  if (ext_target_ptr_) {
    target_in = target_scale_ * (*ext_target_ptr_);
  }

  // ウォッチドッグ: 一定時間コマンドが来ていない場合は即時停止（安全優先）
  // - 目標のスムージング/加速度リミットを介さず、duty=0に即時設定
  // - PI積分器をリセットして惰性での残留出力を防止
  if (cmd_timeout_ms_ > 0 && last_cmd_ms_ptr_ != nullptr) {
    const uint32_t age_ms = now_ms - *last_cmd_ms_ptr_;
    if (age_ms > cmd_timeout_ms_) {
      // 目標・内部状態をクリア
      target_in = cmd_fallback_rad_s_;
      target_filtered_rad_s_ = 0.0f;
      target_rad_s_ = 0.0f;
      integ_ = 0.0f;
      duty_ = 0.0f;
      motor_.setDuty(0.0f);

      // ステータス出力（位置/速度）は従来通り更新
      const float pos = enc_.positionRad();
      const float vel = enc_.velocityRads();
      if (ext_pos_ptr_ && ext_vel_ptr_) {
        *ext_pos_ptr_ = pos;
        *ext_vel_ptr_ = vel;
      }
      return; // 即時復帰（以降のLPF/PI計算は行わない）
    }
  }

  // LPF（一次遅れ）
  float target_shaped = target_in;
  if (target_lpf_tau_s_ > 0.0f && dt_s > 0.0f) {
    // 除算削減: k = dt_s * (1/τ)
    float k = dt_s * inv_target_lpf_tau_;
    if (k > 1.0f) k = 1.0f;
    target_filtered_rad_s_ += k * (target_in - target_filtered_rad_s_);
    target_shaped = target_filtered_rad_s_;
  } else {
    target_filtered_rad_s_ = target_in;
  }

  // 加速度リミット（slew rate limit）
  if (max_acc_rad_s2_ > 0.0f && dt_s > 0.0f) {
    const float max_step = max_acc_rad_s2_ * dt_s;
    const float delta = target_shaped - target_rad_s_;
    if (delta >  max_step) target_shaped = target_rad_s_ + max_step;
    if (delta < -max_step) target_shaped = target_rad_s_ - max_step;
  }

  setTargetRadPerSec(target_shaped);

  if (dt_s > 0.0f) {
    update(dt_s);
  }

  const float pos = enc_.positionRad();
  const float vel = enc_.velocityRads();
  float pos_out = pos;
  if (wrap_pos_pi_) {
    pos_out = wrap_pi(pos_out);
  }
  // debug (counts-based):
  // const float pos = static_cast<float>(enc_.positionCounts());
  // const float vel = enc_.velocityCps();
  if (ext_pos_ptr_ && ext_vel_ptr_) {
    *ext_pos_ptr_ = pos_out;
    *ext_vel_ptr_ = vel;
  }
}
