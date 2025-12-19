#pragma once

#include <stdint.h>

class DinHal {
public:
  bool begin() {
    initialized_ = true;
    first_read_ = true;
    last_mask_ = 0;
    return true;
  }

  void set_mask(uint8_t mask) { mask_ = mask; }

  uint8_t read_all(uint8_t* rising_edges = nullptr, uint8_t* falling_edges = nullptr) {
    if (!initialized_) {
      begin();
    }
    if (rising_edges != nullptr) {
      *rising_edges = first_read_ ? 0 : static_cast<uint8_t>((~last_mask_) & mask_);
    }
    if (falling_edges != nullptr) {
      *falling_edges = first_read_ ? 0 : static_cast<uint8_t>(last_mask_ & ~mask_);
    }
    last_mask_ = mask_;
    first_read_ = false;
    return mask_;
  }

  uint8_t last_mask() const { return last_mask_; }

private:
  bool initialized_ {false};
  bool first_read_ {true};
  uint8_t mask_ {0};
  uint8_t last_mask_ {0};
};
