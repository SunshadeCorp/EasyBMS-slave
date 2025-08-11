#pragma once

#include <WiFi.h>
#include <PubSubClient.h>
#include <WString.h>

#include <map>

#include "mqtt_client_interface.hpp"

#define SSL_ENABLED false

class MqttClient : public IMqttClient {
   public:
    MqttClient(String server, uint16_t port);
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

   private:
    void pub_sub_client_callback(char* topic, uint8_t* payload, unsigned int length);

#if SSL_ENABLED
    NetworkClientSecure _espClient;
#else
    NetworkClient _espClient;
#endif

    PubSubClient _client;
    String _server;
    uint16_t _port;
    String _id;
    String _user;
    String _password;
    String _will_topic;
    uint8_t _will_qos;
    bool _will_retain;
    String _will_message;
    bool _use_will;
    std::map<String, MqttCallback> _mqtt_callbacks;
};