#include "core/network_manager.h"

#include <Arduino.h>

AppNetworkManager* AppNetworkManager::self_ = nullptr;

AppNetworkManager::AppNetworkManager() {
  self_ = this;
}

bool AppNetworkManager::begin() {
#if NUCRYO_USE_ETH_W5500
  Network.onEvent(AppNetworkManager::handle_event);
  return start_eth();
#else
  return false;
#endif
}

void AppNetworkManager::handle_event(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;
  if (self_) {
    self_->on_event(event);
  }
}

void AppNetworkManager::on_event(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[eth] start");
      ETH.setHostname("nu-cryo-esp32s3");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("[eth] link up");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("[eth] got ip: ");
      Serial.println(ETH.localIP());
      eth_connected_ = true;
      break;

#ifdef ARDUINO_EVENT_ETH_LOST_IP
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("[eth] lost ip");
      eth_connected_ = false;
      break;
#endif

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[eth] link down");
      eth_connected_ = false;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("[eth] stop");
      eth_connected_ = false;
      break;

    default:
      break;
  }
}

bool AppNetworkManager::start_eth() {
#if NUCRYO_USE_ETH_W5500
  // Waveshare W5500 is on dedicated SPI pins.
  SPI.begin(BOARD_PINS.w5500_sck, BOARD_PINS.w5500_miso, BOARD_PINS.w5500_mosi);

  // Extra hard reset pulse (helps with "w5500_reset(): reset timeout" cases)
  if (BOARD_PINS.w5500_rst >= 0) {
    pinMode(BOARD_PINS.w5500_rst, OUTPUT);
    digitalWrite(BOARD_PINS.w5500_rst, LOW);
    delay(50);
    digitalWrite(BOARD_PINS.w5500_rst, HIGH);
    delay(150);
  }

  // phy_addr is typically 1 in Arduino-ESP32 W5500 examples.
  bool ok = ETH.begin(ETH_PHY_W5500, /*phy_addr*/ 1,
                      BOARD_PINS.w5500_cs, BOARD_PINS.w5500_int, BOARD_PINS.w5500_rst,
                      SPI);

  if (!ok) {
    Serial.println("[eth] ETH.begin() failed (W5500 init) — continuing without network");
    eth_connected_ = false;
    return false;
  }

  const bool static_requested =
    (NET_DEFAULTS.static_ip.a | NET_DEFAULTS.static_ip.b | NET_DEFAULTS.static_ip.c | NET_DEFAULTS.static_ip.d) != 0;
  if (static_requested) {
    IPAddress local(NET_DEFAULTS.static_ip.a, NET_DEFAULTS.static_ip.b, NET_DEFAULTS.static_ip.c, NET_DEFAULTS.static_ip.d);
    IPAddress gateway(NET_DEFAULTS.static_gw.a, NET_DEFAULTS.static_gw.b, NET_DEFAULTS.static_gw.c, NET_DEFAULTS.static_gw.d);
    IPAddress subnet(NET_DEFAULTS.static_mask.a, NET_DEFAULTS.static_mask.b, NET_DEFAULTS.static_mask.c, NET_DEFAULTS.static_mask.d);
    const bool configured = ETH.config(local, gateway, subnet, gateway);
    eth_static_ = configured;

    if (configured) {
      Serial.print("[eth] static addressing ");
      Serial.println(local);
    } else {
      Serial.println("[eth] ETH.config() failed (static) — using DHCP");
    }
  } else {
    Serial.println("[eth] DHCP (no static IP configured)");
    eth_static_ = false;
  }
#endif
  return true;
}
