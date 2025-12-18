#pragma once

#include <ETH.h>
#include <Network.h>
#include <NetworkClient.h>
#include <SPI.h>

#include "app/app_config.h"

// Manages Ethernet bring-up and link/IP state for MQTT + health monitoring.
class NetworkManager {
public:
  NetworkManager();

  bool begin();

  bool connected() const { return eth_connected_; }
  bool using_static_ip() const { return eth_static_; }
  IPAddress local_ip() const { return ETH.localIP(); }

  NetworkClient& client() { return net_client_; }

private:
  static void handle_event(arduino_event_id_t event, arduino_event_info_t info);
  void on_event(arduino_event_id_t event);
  bool start_eth();

  static NetworkManager* self_;

  bool eth_connected_ {false};
  bool eth_static_ {false};
  NetworkClient net_client_;
};

