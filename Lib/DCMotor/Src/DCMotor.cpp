#include "DCMotor.hpp"

// PWMタイマ/方向ピン/減速比を束ねたDCモータの初期化
DCMotor::DCMotor(TIM_HandleTypeDef *htim, uint16_t channel,
                 GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin,
                 bool direction, float gear_ratio, uint8_t device_id) {
  this->htim = htim;
  this->channel = channel;
  this->direction = direction;
  this->GPIO_Port = GPIO_Port;
  this->GPIO_Pin = GPIO_Pin;
  this->pwm_resolution = __HAL_TIM_GET_AUTORELOAD(htim);
  this->gear_ratio_ = (gear_ratio > 0.0f ? gear_ratio : 1.0f);
  this->device_id_ = device_id;
}

// PWM出力の開始（ARR確定後に分解能を再取得）
void DCMotor::start() {
  // 安全のため、開始前に明示的に停止状態をセット
  __HAL_TIM_SET_COMPARE(htim, channel, 0);
  HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET); // DIR=LOWを既定とする（安全側）
  // PWM開始後にARRが確定する可能性があるため再取得
  HAL_TIM_PWM_Start(htim, channel);
  uint16_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  if (arr != 0) {
    pwm_resolution = arr;
  }
}

// デューティ[-1..1]適用（方向ピンとCCRを設定）
void DCMotor::setDuty(float duty) {
  if (duty > 1.0f) {
    duty = 1.0f;
  } else if (duty < -1.0f) {
    duty = -1.0f;
  }

  float mag = duty >= 0.0f ? duty : -duty; // |duty|

  // 構築順序の関係で初期ARRが0の可能性に対処
  if (pwm_resolution == 0) {
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    if (arr != 0) pwm_resolution = arr;
  }

  uint16_t value = (uint16_t)(pwm_resolution * mag);

  // duty==0 のときは方向ピンを変えない（立上がり時の意図しない回転を防止）
  if (duty > 0.0f) {
    setDirection(1);
  } else if (duty < 0.0f) {
    setDirection(0);
  }

  __HAL_TIM_SET_COMPARE(htim, channel, value);
  // 設定した最終デューティを保持
  last_duty_ = duty;
}

// 方向ピンを所定の極性に設定
void DCMotor::setDirection(bool direction) {
  if (direction == this->direction) {
    HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
  }
}
