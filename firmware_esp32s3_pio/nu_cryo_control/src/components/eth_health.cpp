#include "components/eth_health.h"

#include <Arduino.h>

void EthHealthComponent::configure(bool expected, bool required) {
  rep_.expected = expected;
  rep_.required = required;
  rep_.status = expected ? HealthStatus::MISSING : HealthStatus::OK;
  rep_.severity = expected ? Severity::WARN : Severity::INFO;
  rep_.reason = expected ? "init" : "n/a";
  rep_.since_ms = millis();
  rep_.last_ok_ms = 0;
}

bool EthHealthComponent::probe(uint32_t now_ms) {
  return tick(now_ms);
}

bool EthHealthComponent::tick(uint32_t now_ms) {
  if (!rep_.expected) {
    rep_.status = HealthStatus::OK;
    rep_.severity = Severity::INFO;
    rep_.reason = "disabled";
    rep_.since_ms = now_ms;
    return true;
  }

  if (network_.connected()) {
    if (rep_.status != HealthStatus::OK) rep_.since_ms = now_ms;
    rep_.status = HealthStatus::OK;
    rep_.severity = Severity::INFO;
    rep_.reason = "up";
    rep_.last_ok_ms = now_ms;
    return true;
  }

  if (rep_.status == HealthStatus::OK) rep_.since_ms = now_ms;
  rep_.status = HealthStatus::MISSING;
  rep_.severity = rep_.required ? Severity::CRIT : Severity::WARN;
  rep_.reason = "down";
  return false;
}

