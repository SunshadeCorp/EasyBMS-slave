#pragma once

#include <functional>

#include "Wstring.h"

using MqttCallback = std::function<void(const String&, const String&)>;

class IMqttClient {
   public:
    virtual bool publish(const String &topic, const char* value) = 0;

    virtual bool subscribe(String topic, MqttCallback callback) = 0;
    virtual void disconnect() = 0;
    virtual bool connected() = 0;
    virtual bool connect() = 0;
    virtual bool loop() = 0;
    virtual void set_password(String password) = 0;
    virtual void set_user(String user) = 0;
    virtual void set_id(String id) = 0;
    virtual void set_will(String topic, uint8_t qos, bool retain, String message) = 0;
    virtual String state_string() = 0;

    template <typename T>
    bool publish(const String &topic, T value) {
        return publish(topic, String(value).c_str());
    }

    bool publish(const String &topic, String value) {
        return publish(topic, value.c_str());
    }
};