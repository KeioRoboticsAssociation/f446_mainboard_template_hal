#include "PositionPIDController.hpp"
#include "stm32f4xx_hal.h"
#include <math.h>

void PositionPIDController::reset() {
  integ_ = 0.0f;
  prev_e_ = 0.0f;
  last_vel_cmd_ = 0.0f;
  last_target_pos_ = 0.0f;
  last_meas_pos_ = enc_.positionRad();
  in_reset_zone_ = false;
}

void PositionPIDController::onTimerTick() {
  // Initialize DWT cycle counter once
  if (!dwt_init_) {
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
      DWT->CYCCNT = 0;
      DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }
    prev_cyc_ = DWT->CYCCNT;
    dwt_init_ = true;
    return; // wait for next tick to get dt
  }

  const uint32_t now_cyc = DWT->CYCCNT;
  const uint32_t dt_cyc  = now_cyc - prev_cyc_;
  prev_cyc_ = now_cyc;
  if (dt_cyc == 0) return;
  // 除算削減: dt_s = dt_cyc * inv(SystemCoreClock)
  static float inv_sysclk = 0.0f;
  if (inv_sysclk == 0.0f) inv_sysclk = 1.0f / static_cast<float>(SystemCoreClock);
  const float dt_s = static_cast<float>(dt_cyc) * inv_sysclk;
  if (dt_s <= 0.0f) return;

  // Watchdog: stop (0 vel) if stale
  if (cmd_timeout_ms_ > 0 && last_cmd_ms_ptr_ != nullptr) {
    const uint32_t age_ms = HAL_GetTick() - *last_cmd_ms_ptr_;
    if (age_ms > cmd_timeout_ms_) {
      // Zero velocity and reset integrator
      integ_ = 0.0f;
      prev_e_ = 0.0f;
      last_vel_cmd_ = 0.0f;
      last_target_pos_ = 0.0f;
      last_meas_pos_ = enc_.positionRad();
      in_reset_zone_ = true;
      if (vel_out_ptr_) *vel_out_ptr_ = 0.0f;
      return;
    }
  }

  if (tgt_pos_ptr_ == nullptr || vel_out_ptr_ == nullptr) {
    last_meas_pos_ = enc_.positionRad();
    in_reset_zone_ = false;
    return;
  }

  // Measure position using existing encoder unit convention
  const float pos = enc_.positionRad();
  last_meas_pos_ = pos;
  float tgt = (*tgt_pos_ptr_);
  // Optional: clamp target position to limits (for linear axes)
  if (pos_limits_enable_) {
    if (tgt < pos_min_pos_) tgt = pos_min_pos_;
    if (tgt > pos_max_pos_) tgt = pos_max_pos_;
  }
  last_target_pos_ = tgt;
  float e;
  if (theta_wrap_) {
    if (stateful_dir_enable_) {
      e = computeThetaErrorStateful(pos, tgt);
    } else {
      e = wrap_pi(tgt - pos);
    }
  } else {
    e = tgt - pos;
  }

  // Reset zone behavior (ROS implementation parity)
  if (fabsf(e) < reset_threshold_) {
    reset();
    last_target_pos_ = tgt;
    last_meas_pos_ = pos;
    in_reset_zone_ = true;
    *vel_out_ptr_ = 0.0f;
    return;
  }
  in_reset_zone_ = false;

  // PID compute (error derivative on error)
  integ_ += e * dt_s;
  // 微分は (e - prev)/dt。1回の除算に集約し、あとは乗算。
  const float inv_dt = 1.0f / dt_s;
  const float der = (e - prev_e_) * inv_dt;
  prev_e_ = e;

  float u = kp_ * e + ki_ * integ_ + kd_ * der;
  if (max_vel_ > 0.0f) {
    u = clamp(u, -max_vel_, +max_vel_);
  }
  // 角度加速度制限（出力速度のスルーレート制限）
  if (max_accel_ > 0.0f) {
    const float max_du = max_accel_ * dt_s;
    const float du = u - last_vel_cmd_;
    if (du >  max_du) u = last_vel_cmd_ + max_du;
    if (du < -max_du) u = last_vel_cmd_ - max_du;
  }

  last_vel_cmd_ = u;
  *vel_out_ptr_ = u;
}

// (no-go support removed)

// === Stateful direction selection (no no-go). Locks CW/CCW to avoid crossing 0 unintentionally.
float PositionPIDController::computeThetaErrorStateful(float pos_rad, float tgt_rad) {
  // Normalize for distance computations
  const float p = norm_2pi(pos_rad);
  const float t = norm_2pi(tgt_rad);

  // Target jump detection → unlock
  const float tgt_jump = wrap_pi(tgt_rad - last_tgt_norm_);
  if (fabsf(tgt_jump) > tgt_jump_thr_) {
    dir_lock_ = 0;
    last_tgt_norm_ = t;
  }

  // Distances
  const float dccw = d_ccw(p, t);                  // [0, 2π)
  const float dcw  = 2.0f * static_cast<float>(M_PI) - dccw; // [0, 2π)

  // If locked, check unlock condition by absolute shortest error |wrap(tgt-pos)|
  if (dir_lock_ != 0) {
    const float half_pi = 0.5f * static_cast<float>(M_PI);
    const float e_abs = fabsf(wrap_pi(tgt_rad - pos_rad));
    if (e_abs <= half_pi) {
      dir_lock_ = 0; // unlock and re-decide below
    }
  }

  if (dir_lock_ != 0) {
    float e = (dir_lock_ > 0) ? (+dccw) : (-dcw);
    // Clip magnitude to π for PID niceness
    const float pi = static_cast<float>(M_PI);
    if (e >  pi) e =  pi;
    if (e < -pi) e = -pi;
    // Arrival → unlock
    if (fabsf(e) < settle_err_) {
      dir_lock_ = 0;
    }
    return e;
  }

  // Not locked: by default use shortest-arc error without locking.
  // Lock only when "zero-cross avoidance" enforced conditions are met.
  float e_short = wrap_pi(tgt_rad - pos_rad); // [-π, π]
  // Near target (|e| <= π/2) は常に短径を優先し、長回りの強制を適用しない
  const float half_pi_guard = 0.5f * static_cast<float>(M_PI);
  if (fabsf(e_short) <= half_pi_guard) {
    return e_short;
  }

  // Apply rule: avoid crossing 0 with asymmetric thresholds (enforce-only)
  const float pos_wrapped = wrap_pi(pos_rad); // [-π, π]
  const float pi = static_cast<float>(M_PI);
  const float half_pi = 0.5f * pi;
  const float three_half_pi = 1.5f * pi;
  const float eps = 1e-3f; // guard equality noise
  bool enforce_ccw = false;
  bool enforce_cw  = false;
  if (pos_wrapped >= 0.0f && pos_wrapped <= half_pi) {
    if (dccw >= (pi - eps) && dccw <= (three_half_pi + eps)) enforce_ccw = true;
  } else if (pos_wrapped <= 0.0f && pos_wrapped >= -half_pi) {
    if (dcw >= (pi - eps)) enforce_cw = true; // 過去の指示: CWがπ以上ならCWを強制
  }

  if (enforce_ccw) {
    dir_lock_ = +1;
    last_tgt_norm_ = t;
    float e = +dccw;
    if (e >  pi) e =  pi;
    if (e < -pi) e = -pi;
    return e;
  }
  if (enforce_cw) {
    dir_lock_ = -1;
    last_tgt_norm_ = t;
    float e = -dcw;
    if (e >  pi) e =  pi;
    if (e < -pi) e = -pi;
    return e;
  }

  // Otherwise, do not lock; just use shortest-arc error (will smoothly go to zero)
  return e_short;
}
