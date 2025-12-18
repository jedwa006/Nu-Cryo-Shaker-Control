#include "components/din.h"

#include <Arduino.h>

namespace {
bool bit_set(uint8_t mask, uint8_t bit) { return (mask & (1u << bit)) != 0; }
}

DinComponent::DinComponent(DinHal& hal) : hal_(hal) {}

void DinComponent::configure(bool expected, bool required) {
  rep_.expected = expected;
  rep_.required = required;
  rep_.status = expected ? HealthStatus::MISSING : HealthStatus::UNCONFIGURED;
  rep_.severity = required ? Severity::CRIT : Severity::INFO;
  rep_.reason = expected ? "not_probed" : "unconfigured";
  rep_.since_ms = 0;
  rep_.last_ok_ms = 0;
}

bool DinComponent::probe(uint32_t now_ms) {
  if (!rep_.expected) return false;
  initialized_ = hal_.begin();
  snapshot_.mask = hal_.read_all(&snapshot_.rising, &snapshot_.falling);
  update_health(now_ms);
  return interlocks_ok();
}

bool DinComponent::tick(uint32_t now_ms) {
  if (!rep_.expected) return false;
  if (!initialized_) return probe(now_ms);

  snapshot_.mask = hal_.read_all(&snapshot_.rising, &snapshot_.falling);
  update_health(now_ms);
  return interlocks_ok();
}

bool DinComponent::interlocks_ok() const {
  const bool estop_ok = bit_set(snapshot_.mask, BIT_ESTOP_OK);
  const bool lid_locked = bit_set(snapshot_.mask, BIT_LID_LOCKED);
  const bool door_closed = bit_set(snapshot_.mask, BIT_DOOR_CLOSED);
  return estop_ok && lid_locked && door_closed;
}

void DinComponent::update_health(uint32_t now_ms) {
  const bool estop_ok = bit_set(snapshot_.mask, BIT_ESTOP_OK);
  const bool lid_locked = bit_set(snapshot_.mask, BIT_LID_LOCKED);
  const bool door_closed = bit_set(snapshot_.mask, BIT_DOOR_CLOSED);
  const bool ok = estop_ok && lid_locked && door_closed;

  const char* reason = "ok";
  if (!estop_ok) {
    reason = "estop_tripped";
  } else if (!door_closed) {
    reason = "door_open";
  } else if (!lid_locked) {
    reason = "lid_unlocked";
  }

  if (ok) {
    rep_.status = HealthStatus::OK;
    rep_.severity = Severity::INFO;
    rep_.reason = "ok";
    rep_.last_ok_ms = now_ms;
    if (rep_.since_ms == 0) rep_.since_ms = now_ms;
  } else {
    if (rep_.since_ms == 0 || rep_.status == HealthStatus::OK) rep_.since_ms = now_ms;
    rep_.status = HealthStatus::ERROR;
    rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
    rep_.reason = reason;
  }
}
