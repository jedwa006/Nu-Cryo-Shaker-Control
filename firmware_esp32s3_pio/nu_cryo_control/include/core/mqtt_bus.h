    #pragma once
    #include <Arduino.h>
    #include <PubSubClient.h>
    #include <ArduinoJson.h>

    class MqttBus {
    public:
      using MessageHandler = std::function<void(const char* topic, const uint8_t* payload, size_t len)>;

      bool begin(PubSubClient& client, const char* machine_id, const char* node_id);
      void set_handler(MessageHandler h) { handler_ = std::move(h); }

      void loop();
      bool ensure_connected();

      // Publish JSON (small messages). For bulk, use chunked byte payloads (not implemented here).
      bool publish_json(const char* subtopic, const JsonDocument& doc, bool retained=false, int qos=0);

      // Subscribe to subtopic under root
      bool subscribe(const char* subtopic, int qos=0);

      const char* root() const { return root_; }

    private:
      PubSubClient* client_ {nullptr};
      char root_[96] {};
      MessageHandler handler_ {};

      static void _callback_shim(char* topic, uint8_t* payload, unsigned int length);
      static MqttBus* self_;
    };
