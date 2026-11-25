#pragma once

#include <PsychicMqttClient.h>
#include <WString.h>

#include <memory>
#include <vector>

#include "balancer_interface.hpp"
#include "bms.hpp"

#include "freertos/semphr.h"

class BMS;

using time_ms = unsigned long;

class MqttAdapter : public IBalancer {
   public:
    MqttAdapter(const std::shared_ptr<BMS> &bms, int index = 0);

    void init(const char *const server, const char *const user,
              const char *const password);
    void loop();
    std::vector<bool> balance(const std::vector<float> &voltages) override;
    void set_ota_server_cert(const char *const ota_server, const char *const cert);

   private:
    std::shared_ptr<BMS> _bms;
    PsychicMqttClient _mqtt;
    static constexpr time_ms MASTER_TIMEOUT = 5000;
    String _mac_topic, _module_topic, _will_topic, _client_id;
    time_ms _last_master_uptime;
    SemaphoreHandle_t _balance_semaphore;
    std::vector<time_ms> _balance_start_time;
    std::vector<time_ms> _balance_duration;
    const char *_ota_cert;
    const char *_ota_server;
    bool _current_measurement;
    int _index;
    float _zero_current;

    void on_mqtt_connect(bool session_present);
    void on_mqtt_master_uptime(const String &topic_string, const String &payload_string);
    void on_mqtt_balance_request(const String &topic_string, const String &payload_string);
    void on_mqtt_blink(const String &topic_string, const String &payload_string);
    void on_mqtt_restart(const String &topic_string, const String &payload_string);
    void on_mqtt_set_config(const String &topic_string, const String &payload_string);
    void on_mqtt_ota(const String &topic_string, const String &payload_string);
    String cpu_description() const;
    String flash_description() const;
    void reset_balancing(size_t size);
    void publish(const String &topic);

    template <typename T>
    void publish_value(const String &topic, const T &value) {
        String payload(value);
        _mqtt.publish(topic.c_str(), 0, false, payload.c_str(), 0, false);
    }
};

template <>
inline void MqttAdapter::publish_value<String>(const String &topic, const String &value) {
    _mqtt.publish(topic.c_str(), 0, false, value.c_str(), 0, false);
}