#pragma once
#include "stm32f4xx_hal.h"
#include <cstdint>

// 単純なGPIO駆動のエア電磁弁ドライバ
// - begin(): 初期状態へ（安全のためOFF）
// - set(true/false): ON/OFF 切替
// - on()/off(): 明示的制御
// - deviceID(): 任意の識別子（0/1など）
class AirSolenoid {
public:
  // コンストラクタ: 対象GPIOポート/ピン、論理極性（active_high）、任意のデバイスID
  AirSolenoid(GPIO_TypeDef* port, uint16_t pin, bool active_high = true, uint8_t device_id = 0);

  // 初期化: 出力を安全側（OFF）へ
  void begin();

  // true=ON, false=OFF
  void set(bool on);

  // 明示的ON/OFF（極性はactive_highに従う）
  void on();
  void off();

  // 任意の識別子（配線系のスロット識別など）
  uint8_t deviceID() const;

private:
  GPIO_TypeDef* port_;
  uint16_t      pin_;
  bool          active_high_;
  uint8_t       device_id_;
};
