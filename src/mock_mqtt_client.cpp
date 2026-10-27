#include "mock_mqtt_client.hpp"

#include "debug.hpp"

MockMqttClient::MockMqttClient() {
    id = "";
    user = "";
    password = "";
    will_topic = "";
    will_qos = 0;
    will_retain = true;
    will_message = "";

    connect_result = true;
    loop_result = true;
}

bool MockMqttClient::connect() {
    DEBUG_PRINTLN("MQTT Mock: connect()");
    is_connected = connect_result;
    return connect_result;
}

void MockMqttClient::set_will(const String &topic, uint8_t qos, bool retain, const String & message) {
    will_topic = topic;
    will_qos = qos;
    will_retain = retain;
    will_message = message;
}

bool MockMqttClient::publish(const String &topic, const char* value) {
    DEBUG_PRINTLN("MQTT Mock: publish(): " + topic + " (" + value + ")");
    return publish_result;
}

void MockMqttClient::disconnect() {
    DEBUG_PRINTLN("MQTT Mock: disconnect()");
}

bool MockMqttClient::connected() {
    return is_connected;
}

void MockMqttClient::receive_messagge(String topic, String payload) {
    DEBUG_PRINTLN("MQTT Mock: receive_message(): " + topic + " (" + payload + ")");
    auto callback_it = mqtt_callbacks.find(topic);
    if (callback_it != mqtt_callbacks.end()) {
        callback_it->second(topic, payload);
    } else {
        DEBUG_PRINTLN("TEST ERROR: MockMqttClient was not subscribed to " + topic);
    }
}

bool MockMqttClient::subscribe(const String & topic, MqttCallback callback) {
    DEBUG_PRINTLN("MQTT Mock: subscribe(): " + topic);
    mqtt_callbacks[topic] = callback;
    return true;
}