#include "mqtt_client.hpp"

#include "debug.hpp"

MqttClient::MqttClient(String server, uint16_t port) :
    _client(server.c_str(), port, _espClient),
    _server(server),
    _port{port},
    _id(),
    _user(),
    _password(),
    _will_topic(),
    _will_qos{0},
    _will_retain{true},
    _will_message(),
    _use_will{false},
    _mqtt_callbacks{}
{
    _client.setCallback([this](char* a, uint8_t* b, unsigned int c) { pub_sub_client_callback(a, b, c); });
}

void MqttClient::set_password(String password) {
    _password = password;
}

void MqttClient::set_user(String user) {
    _user = user;
}

void MqttClient::set_id(String id) {
    _id = id;
}

bool MqttClient::connect() {
    _client.setServer(_server.c_str(), _port);
    if (_use_will) {
        return _client.connect(_id.c_str(), _user.c_str(), _password.c_str(), _will_topic.c_str(), _will_qos, _will_retain, _will_message.c_str());
    } else {
        return _client.connect(_id.c_str(), _user.c_str(), _password.c_str());
    }
}

void MqttClient::set_will(String topic, uint8_t qos, bool retain, String message) {
    _will_topic = topic;
    _will_qos = qos;
    _will_retain = retain;
    _will_message = message;
    _use_will = true;
}

bool MqttClient::publish(const String &topic, const char* value) {
    return _client.publish(topic.c_str(), value, true);
}

void MqttClient::disconnect() {
    _client.disconnect();
}

bool MqttClient::connected() {
    return _client.connected();
}

bool MqttClient::loop() {
    return _client.loop();
}

String MqttClient::state_string() {
    auto state = _client.state();

    switch (state) {
        case MQTT_CONNECTION_TIMEOUT:
            return "MQTT_CONNECTION_TIMEOUT";
        case MQTT_CONNECTION_LOST:
            return "MQTT_CONNECTION_LOST";
        case MQTT_CONNECT_FAILED:
            return "MQTT_CONNECT_FAILED";
        case MQTT_DISCONNECTED:
            return "MQTT_DISCONNECTED";
        case MQTT_CONNECTED:
            return "MQTT_CONNECTED";
        case MQTT_CONNECT_BAD_PROTOCOL:
            return "MQTT_CONNECT_BAD_PROTOCOL";
        case MQTT_CONNECT_BAD_CLIENT_ID:
            return "MQTT_CONNECT_BAD_CLIENT_ID";
        case MQTT_CONNECT_UNAVAILABLE:
            return "MQTT_CONNECT_UNAVAILABLE";
        case MQTT_CONNECT_BAD_CREDENTIALS:
            return "MQTT_CONNECT_BAD_CREDENTIALS";
        case MQTT_CONNECT_UNAUTHORIZED:
        default:
            return "undefined";
    }
}

void MqttClient::pub_sub_client_callback(char* topic, uint8_t* payload, unsigned int length) {
    String topic_string = String(topic);
    String payload_string = String();
    payload_string.concat((char*)payload, length);
    auto it = _mqtt_callbacks.find(topic_string);

    if (it != _mqtt_callbacks.end()) {
        it->second(topic_string, payload_string);
    }
}

bool MqttClient::subscribe(String topic, MqttCallback callback) {
    _mqtt_callbacks[topic] = callback;
    return _client.subscribe(topic.c_str());
}