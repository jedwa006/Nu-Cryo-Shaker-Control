\
    #include "mqtt_bus.h"

    MqttBus* MqttBus::self_ = nullptr;

    bool MqttBus::begin(PubSubClient& client, const char* machine_id, const char* node_id) {
      client_ = &client;
      snprintf(root_, sizeof(root_), "%s/%s", machine_id, node_id);
      self_ = this;
      client_->setCallback(&_callback_shim);
      return true;
    }

    void MqttBus::_callback_shim(char* topic, uint8_t* payload, unsigned int length) {
      if (!self_ || !self_->handler_) return;
      self_->handler_(topic, payload, length);
    }

    void MqttBus::loop() {
      if (client_) client_->loop();
    }

    bool MqttBus::ensure_connected() {
      if (!client_) return false;
      if (client_->connected()) return true;

      // Caller should have configured server + network already.
      // Keep reconnect behavior simple; add backoff in main loop if needed.
      char client_id[48];
      snprintf(client_id, sizeof(client_id), "nucryo-%lu", (unsigned long)ESP.getEfuseMac());

      // NOTE: Credentials are set in main (PubSubClient connect overload).
      // This function only returns connection status.
      return client_->connected();
    }

    bool MqttBus::publish_json(const char* subtopic, const JsonDocument& doc, bool retained, int /*qos*/) {
      if (!client_ || !client_->connected()) return false;

      char topic[160];
      snprintf(topic, sizeof(topic), "%s/%s", root_, subtopic);

      // Serialize into a stack buffer. If you later move to CBOR, the signature stays.
      char buf[1024];
      const size_t n = serializeJson(doc, buf, sizeof(buf));
      if (n == 0 || n >= sizeof(buf)) return false;

      return client_->publish(topic, (const uint8_t*)buf, n, retained);
    }

    bool MqttBus::subscribe(const char* subtopic, int /*qos*/) {
      if (!client_) return false;
      char topic[160];
      snprintf(topic, sizeof(topic), "%s/%s", root_, subtopic);
      return client_->subscribe(topic);
    }
