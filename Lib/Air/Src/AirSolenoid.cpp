#include "AirSolenoid.hpp"

AirSolenoid::AirSolenoid(GPIO_TypeDef* port, uint16_t pin, bool active_high, uint8_t device_id)
: port_(port), pin_(pin), active_high_(active_high), device_id_(device_id) {}

void AirSolenoid::begin() {
  // ピンはCubeMXで出力設定済みを想定。初期状態は安全側（OFF）。
  off();
}

void AirSolenoid::set(bool on) { on ? this->on() : this->off(); }

void AirSolenoid::on()  { HAL_GPIO_WritePin(port_, pin_, active_high_ ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void AirSolenoid::off() { HAL_GPIO_WritePin(port_, pin_, active_high_ ? GPIO_PIN_RESET : GPIO_PIN_SET); }

uint8_t AirSolenoid::deviceID() const { return device_id_; }

