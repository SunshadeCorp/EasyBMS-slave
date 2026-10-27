#pragma once

#include <WString.h>

#include <map>
#include <functional>

#define SSL_ENABLED false

using MqttCallback = std::function<void(const String&, const String&)>;

class MockMqttClient {
   public:
    MockMqttClient();
    bool publish(const String &topic, const char* value);
    bool subscribe(const String & topic, MqttCallback callback);
    void disconnect();
    bool connected();
    bool connect();
    void set_will(const String &topic, uint8_t qos, bool retain, const String &message);

    void receive_messagge(String topic, String payload);

   public:
    String id;
    String user;
    String password;
    String will_topic;
    uint8_t will_qos;
    bool will_retain;
    String will_message;

    std::map<String, MqttCallback> mqtt_callbacks;
    bool connect_result;
    bool loop_result;
    bool is_connected;
    bool publish_result;
    String state;
};