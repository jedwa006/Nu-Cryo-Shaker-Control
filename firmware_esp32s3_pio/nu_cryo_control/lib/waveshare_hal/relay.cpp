#include "waveshare_hal/relay.h"

#include <Arduino.h>

#include "waveshare_vendor/I2C_Driver.h"
#include "waveshare_vendor/WS_Relay.h"
#include "waveshare_vendor/WS_TCA9554PWR.h"

bool RelayHal::begin() {
  I2C_Init();
  TCA9554PWR_Init(0x00, 0x00);  // all outputs, default LOW
  initialized_ = true;

  uint8_t mask = 0;
  return read_mask(mask);
}

bool RelayHal::write_mask(uint8_t mask) {
  if (!initialized_) {
    begin();
  }
  return Relay_CHxs_PinState(mask);
}

bool RelayHal::read_mask(uint8_t& mask) {
  if (!initialized_) {
    begin();
  }

  uint8_t data = 0;
  const bool ok = (I2C_Read(TCA9554_ADDRESS, TCA9554_OUTPUT_REG, &data, 1) == 0);
  mask = data;
  return ok;
}
