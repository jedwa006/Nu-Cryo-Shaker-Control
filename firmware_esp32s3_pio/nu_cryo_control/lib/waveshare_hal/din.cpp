#include "waveshare_hal/din.h"

#include <Arduino.h>

#include "waveshare_vendor/WS_DIN.h"

namespace {
constexpr uint8_t kDinPins[8] = {
  DIN_PIN_CH1, DIN_PIN_CH2, DIN_PIN_CH3, DIN_PIN_CH4,
  DIN_PIN_CH5, DIN_PIN_CH6, DIN_PIN_CH7, DIN_PIN_CH8,
};
}

bool DinHal::begin() {
  for (uint8_t pin : kDinPins) {
    pinMode(pin, INPUT_PULLUP);
  }
  initialized_ = true;
  first_read_ = true;
  last_mask_ = 0;
  return true;
}

uint8_t DinHal::read_all(uint8_t* rising_edges, uint8_t* falling_edges) {
  if (!initialized_) {
    begin();
  }

  const uint8_t raw = DIN_Read_CHxs();
  const uint8_t mask = DIN_Inverse_Enable ? static_cast<uint8_t>(~raw) : raw;

  if (rising_edges) {
    *rising_edges = first_read_ ? 0 : static_cast<uint8_t>((~last_mask_) & mask);
  }
  if (falling_edges) {
    *falling_edges = first_read_ ? 0 : static_cast<uint8_t>(last_mask_ & ~mask);
  }

  last_mask_ = mask;
  first_read_ = false;
  return mask;
}
