#include "components/relay_component.h"

#include <Arduino.h>

RelayComponent::RelayComponent(RelayHal& hal) : hal_(hal) {}

void RelayComponent::configure(bool expected, bool required) {
  rep_.expected = expected;
  rep_.required = required;
  rep_.status = expected ? HealthStatus::MISSING : HealthStatus::UNCONFIGURED;
  rep_.severity = required ? Severity::CRIT : Severity::INFO;
  rep_.reason = expected ? "not_probed" : "unconfigured";
  rep_.since_ms = 0;
  rep_.last_ok_ms = 0;
}

bool RelayComponent::probe(uint32_t now_ms) {
  if (!rep_.expected) return false;
  initialized_ = hal_.begin();
  if (!initialized_) {
    set_error(now_ms, "init_failed");
    return false;
  }
  return refresh_state(now_ms);
}

bool RelayComponent::tick(uint32_t now_ms) {
  if (!rep_.expected) return false;
  if (!initialized_) return probe(now_ms);
  return refresh_state(now_ms);
}

bool RelayComponent::set_mask(uint8_t mask, uint32_t now_ms) {
  if (!rep_.expected) return false;
  if (!initialized_ && !probe(now_ms)) return false;

  const bool ok = hal_.write_mask(mask);
  if (ok) {
    mask_ = mask;
    rep_.status = HealthStatus::OK;
    rep_.severity = Severity::INFO;
    rep_.reason = "ok";
    rep_.last_ok_ms = now_ms;
    if (rep_.since_ms == 0) rep_.since_ms = now_ms;
  } else {
    set_error(now_ms, "write_fail");
  }
  return ok;
}

bool RelayComponent::refresh_state(uint32_t now_ms) {
  uint8_t read_mask = 0;
  const bool ok = hal_.read_mask(read_mask);
  if (ok) {
    mask_ = read_mask;
    rep_.status = HealthStatus::OK;
    rep_.severity = Severity::INFO;
    rep_.reason = "ok";
    rep_.last_ok_ms = now_ms;
    if (rep_.since_ms == 0) rep_.since_ms = now_ms;
  } else {
    set_error(now_ms, "read_fail");
  }
  return ok;
}

void RelayComponent::set_error(uint32_t now_ms, const char* reason, HealthStatus status) {
  if (rep_.since_ms == 0 || rep_.status == HealthStatus::OK) rep_.since_ms = now_ms;
  rep_.status = status;
  rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
  rep_.reason = reason;
}
