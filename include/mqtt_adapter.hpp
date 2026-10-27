#pragma once

#include <PsychicMqttClient.h>
#include <WString.h>

#include <memory>
#include <string_view>
#include <vector>

#include "balancer_interface.hpp"
#include "bms.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class BMS;

using time_ms = unsigned long;

class MqttAdapter : public IBalancer {
   public:
    MqttAdapter(const std::shared_ptr<BMS> &bms, const std::string_view &server, const std::string_view &user,
                const std::string_view &password, const String &client_id);

    void init();
    void init(int index);
    void reconnect();
    void loop();
    std::vector<bool> balance(const std::vector<float> &voltages) override;
    void set_ota_server(String ota_server);
    void set_ota_cert(std::string_view cert);
    String module_topic() const;

   private:
    std::shared_ptr<BMS> _bms;
    PsychicMqttClient _mqtt;
    static constexpr time_ms MASTER_TIMEOUT = 5000;
    time_ms _last_connection;
    String _hostname;
    String _mac_topic;
    String _module_topic;
    time_ms _last_master_uptime;
    SemaphoreHandle_t _balance_semaphore;
    std::vector<time_ms> _balance_start_time;
    std::vector<time_ms> _balance_duration;
    std::string_view _ota_cert;
    String _ota_server;
    bool _current_measurement;
    int _index;

    void on_mqtt_connect(bool session_present);
    void on_mqtt_master_uptime(const String &topic_string, const String &payload_string);
    void on_mqtt_balance_request(const String &topic_string, const String &payload_string);
    void on_mqtt_blink(const String &topic_string, const String &payload_string);
    void on_mqtt_restart(const String &topic_string, const String &payload_string);
    void on_mqtt_set_config(const String &topic_string, const String &payload_string);
    void on_mqtt_ota(const String &topic_string, const String &payload_string);
    bool is_uint(const String &number_string) const;
    int cell_id_from_name(const String &cell_name) const;
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