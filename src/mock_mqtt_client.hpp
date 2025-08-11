#pragma once

#include <WString.h>

#include <map>

#include "mqtt_client_interface.hpp"

#define SSL_ENABLED false

using MqttCallback = std::function<void(const String&, const String&)>;

class MockMqttClient : public IMqttClient {
   public:
    MockMqttClient();
    bool publish(const String &topic, const char* value) override;
    bool subscribe(String topic, MqttCallback callback) override;
    void disconnect() override;
    bool connected() override;
    bool connect() override;
    bool loop() override;
    void set_password(String password) override;
    void set_user(String user) override;
    void set_id(String id) override;
    void set_will(String topic, uint8_t qos, bool retain, String message) override;
    String state_string() override;

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