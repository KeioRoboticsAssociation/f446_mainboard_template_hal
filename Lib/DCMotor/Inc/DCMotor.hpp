#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal_tim.h>

class DCMotor {
public:
  DCMotor(TIM_HandleTypeDef *htim, uint16_t channel, GPIO_TypeDef *GPIO_Port,
          uint16_t GPIO_Pin, bool direction, float gear_ratio, uint8_t device_id = 0);
  void setDuty(float duty);
  void start();
  float gearRatio() const { return gear_ratio_; }
  uint8_t deviceID() const { return device_id_; }
  // 現在適用中のデューティ（[-1,1]）。setDutyで最後に設定された値を返す。
  float getDuty() const { return last_duty_; }
  // 呼びやすい別名（互換用）
  float get_duty() const { return last_duty_; }
  // 正のdutyを与えたときに方向ピンがHIGHになるならtrue（配線極性）
  bool positiveDirectionIsHigh() const { return direction; }
  // 座標系に対する符号の決定に使う簡易符号（+1: 正相関, -1: 逆相関）
  int polaritySign() const { return direction ? +1 : -1; }

private:
  void setDirection(bool direction);
  TIM_HandleTypeDef *htim;
  uint16_t channel;
  bool direction;

  GPIO_TypeDef *GPIO_Port;
  uint16_t GPIO_Pin;
  uint16_t pwm_resolution;
  float    gear_ratio_ = 1.0f; // 出力1回転あたりのモータ回転数（例: 30.0）
  uint8_t  device_id_ = 0;
  float    last_duty_ = 0.0f;   // 直近に適用したデューティ（符号付き）
};

#ifdef __cplusplus
}
#endif
