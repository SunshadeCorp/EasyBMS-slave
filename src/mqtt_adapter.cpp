#include "mqtt_adapter.hpp"

#include <algorithm>
#include <bitset>
#include <WiFi.h>

#include "debug.hpp"
#include "config.h"
#include "version.h"
#include "wifi.hpp"

extern void startCANThread();

MqttAdapter::MqttAdapter(const std::shared_ptr<BMS> &bms, const std::string_view &server, const std::string_view &user,
                         const std::string_view &password, const String &client_id) :
    _bms(bms),
    _mqtt(),
    _last_connection{0},
    _hostname(),
    _mac_topic(),
    _module_topic(),
    _last_master_uptime{0},
    _balance_semaphore{nullptr},
    _balance_start_time{},
    _balance_duration{},
    _ota_cert{nullptr},
    _ota_server(),
    _current_measurement{false},
    _index{0}
{
    _balance_semaphore = xSemaphoreCreateMutex();
    _mqtt.setServer(server.data());
    _mqtt.setCredentials(user.data(), password.data());
    _mqtt.setClientId(client_id.c_str());
    _mqtt.setKeepAlive(60);
    _mqtt.onConnect([this](bool session_present) { on_mqtt_connect(session_present); });
}

bool MqttAdapter::is_uint(const String &number_string) const {
    return !number_string.isEmpty() && std::all_of(number_string.begin(), number_string.end(), isDigit);
}

int MqttAdapter::cell_id_from_name(const String &cell_name) const {
    if (!is_uint(cell_name)) {
        return -1;
    }
    return cell_name.toInt() - 1;
}

String MqttAdapter::cpu_description() const {
    return String(ESP.getChipModel()) + " rev " + ESP.getChipRevision() + " " + ESP.getChipCores() + "x" +
           ESP.getCpuFreqMHz() + "MHz";
}

String MqttAdapter::flash_description() const {
    return String(ESP.getFlashChipSize() / 1024 / 1024) + " MiB, Mode: " + ESP.getFlashChipMode();
}

void MqttAdapter::init() {
    init(0);
}

void MqttAdapter::init(int index) {
    if (index == 0) {
        _mac_topic = String("esp-module/") + WifiManager::mac_string();
    } else {
        _mac_topic = String("esp-module/") + WifiManager::mac_string() + "_" + String(index);
        _index = index;
    }

    _module_topic = _mac_topic;
    _mqtt.setWill((_module_topic + "/available").c_str(), 0, true, "offline");

    if (_bms->mode() == BalanceMode::slave) {
        _mqtt.onTopic("master/uptime", 0, [this](char *topic, char *payload, int, int, bool) {
            on_mqtt_master_uptime(String(topic), String(payload));
                });
        _mqtt.onTopic((_mac_topic + "/blink").c_str(), 0,
                    [this](char *topic, char *payload, int, int, bool) {
                        on_mqtt_blink(String(topic), String(payload));
                    });
    }
    _mqtt.onTopic((_mac_topic + "/set_config").c_str(), 0,
                  [this](char *topic, char *payload, int, int, bool) {
                      on_mqtt_set_config(String(topic), String(payload));
                  });
    _mqtt.onTopic((_mac_topic + "/restart").c_str(), 0,
                  [this](char *topic, char *payload, int, int, bool) {
                      on_mqtt_restart(String(topic), String(payload));
                  });
    _mqtt.onTopic((_mac_topic + "/ota").c_str(), 0,
                  [this](char *topic, char *payload, int, int, bool) {
                      on_mqtt_ota(String(topic), String(payload));
                  });
    _mqtt.connect();
}

void MqttAdapter::reconnect() {
    if (!_mqtt.connected()) {
        DEBUG_PRINT("Attempting MQTT connection...");
        _mqtt.connect();
    }
}

void MqttAdapter::loop() {
    if (!_mqtt.connected()) {
        if (_last_connection != 0 && millis() - _last_connection >= 30000) {
            _bms->restart();
        }
        _mqtt.connect();
        return;
    }

    publish(_module_topic);
    _last_connection = millis();
}

void MqttAdapter::on_mqtt_connect(bool session_present) {
    DEBUG_PRINTLN("connected");
    publish_value(_module_topic + "/available", "online");
    if (!_module_topic.equals(_mac_topic)) {
        publish_value(_mac_topic + "/available", "undefined");
    }
    publish_value(_mac_topic + "/module_topic", _module_topic);
    publish_value(_mac_topic + "/version", VERSION);
    publish_value(_mac_topic + "/build_timestamp", BUILD_TIMESTAMP);
    publish_value(_mac_topic + "/wifi", WiFi.SSID());
    publish_value(_mac_topic + "/ip", WiFi.localIP().toString());
    publish_value(_mac_topic + "/esp_sdk", ESP.getSdkVersion());
    publish_value(_mac_topic + "/cpu", cpu_description());
    publish_value(_mac_topic + "/flash", flash_description());
    publish_value(_mac_topic + "/bms_mode", as_string(_bms->mode()));

    _mqtt.onTopic((_module_topic + "/cell/+/balance_request").c_str(), 0,
                [this](char *topic, char *payload, int, int, bool) {
                    on_mqtt_balance_request(String(topic), String(payload));
                });

}

void MqttAdapter::reset_balancing(size_t size) {
    _balance_start_time.assign(size, false);
    _balance_duration.assign(size, false);
}

std::vector<bool> MqttAdapter::balance(const std::vector<float> &voltages) {
    auto balance_bits = std::vector<bool>(voltages.size(), false);

    if (xSemaphoreTake(_balance_semaphore, 100) == pdFALSE) {
        reset_balancing(balance_bits.size());
        return balance_bits;
    }

    if (_balance_start_time.size() != voltages.size()) {
        reset_balancing(balance_bits.size());
        xSemaphoreGive(_balance_semaphore);
        return balance_bits;
    }

    for (size_t i = 0; i < _balance_start_time.size(); ++i) {
        if (millis() - _balance_start_time[i] <= _balance_duration[i]) {
            balance_bits[i] = true;
        } else if (_balance_duration[i] > 0) {
            _balance_duration[i] = 0;
        }
    }

    xSemaphoreGive(_balance_semaphore);
    return balance_bits;
}

void MqttAdapter::publish(const String &topic) {
    auto monitor = _bms->battery_monitor();
    auto balance_bits = monitor->balance_bits();
    String temperatures;

    publish_value(topic + "/uptime", millis());
    publish_value(topic + "/pec15_error_count", monitor->measure_error_count());
    publish_value(topic + "/battery_config", as_string(monitor->battery_config()));

    if (monitor->measure_error()) {
        return;
    }

    size_t index = 0;
    for (const auto &voltage : monitor->cell_voltages()) {
        String cell_name = String(index + 1);
        publish_value(topic + "/cell/" + cell_name + "/voltage", String(voltage, 3));
        publish_value(topic + "/cell/" + cell_name + "/is_balancing", balance_bits[index] ? "1" : "0");
        ++index;
    }

    publish_value(topic + "/module_voltage", monitor->module_voltage());
    for (const auto &temperature : monitor->module_temps()) {
        temperatures += String(temperature) + ",";
    }
    publish_value(topic + "/module_temps", temperatures);

    temperatures = "";
    for (const auto &temperature : monitor->pcb_temps()) {
        temperatures += String(temperature) + ",";
    }
    publish_value(topic + "/pcb_temps", temperatures);
    publish_value(topic + "/chip_temp", monitor->chip_temp());
    publish_value(topic + "/battery_type", as_string(monitor->battery_type()));
#if LTC_COUNT > 1
    if (Config::bms_mode == BalanceMode::slave) {
        if (_current_measurement && (_index == 0)) {
            publish_value("esp-total/total_current", monitor->battery_current());
            publish_value("esp-total/total_voltage", 4 * monitor->module_voltage()); // TODO: This is a hack, we should get the total voltage from the master instead of calculating it here
        }
    } else if (_index == 0) {
        publish_value(topic + "/battery_current", monitor->battery_current());
    }
#endif
}

String MqttAdapter::module_topic() const {
    return _module_topic;
}

void MqttAdapter::on_mqtt_master_uptime(const String &topic_string, const String &payload_string) {
    DEBUG_PRINT("Got heartbeat from master: ");
    DEBUG_PRINTLN(payload_string);

    _bms->flip_led();
    unsigned long long uptime_u_long = std::stoull(payload_string.c_str());
    if (uptime_u_long - _last_master_uptime > MASTER_TIMEOUT) {
        DEBUG_PRINTLN(uptime_u_long);
        DEBUG_PRINTLN(_last_master_uptime);
        DEBUG_PRINTLN(">>> Master Timeout!!");
    }
    _last_master_uptime = uptime_u_long;
}

void MqttAdapter::on_mqtt_balance_request(const String &topic_string, const String &payload_string) {
    if (xSemaphoreTake(_balance_semaphore, 100) == pdFALSE) {
        return;
    }

    String cell_name = topic_string.substring((_module_topic + "/cell/").length());
    cell_name = cell_name.substring(0, cell_name.indexOf("/"));
    int cell_id = cell_id_from_name(cell_name);
    if (cell_id == -1 || static_cast<size_t>(cell_id) >= _balance_duration.size()) {
        DEBUG_PRINTLN("MQTT: Got invalid cell name for balancing: " + cell_name);
        xSemaphoreGive(_balance_semaphore);
        return;
    }

    time_ms balance_time = payload_string.toInt();
    if (balance_time > 1000 * 60 * 5) {
        DEBUG_PRINTLN("MQTT: Balance time too long (> 5 mins): " + String(balance_time) + "ms");
        xSemaphoreGive(_balance_semaphore);
        return;
    }

    _balance_start_time[cell_id] = millis();
    _balance_duration[cell_id] = balance_time;
    xSemaphoreGive(_balance_semaphore);
}

void MqttAdapter::on_mqtt_blink(const String &topic_string, const String &payload_string) {
    _bms->blink();
}

void MqttAdapter::on_mqtt_restart(const String &topic_string, const String &payload_string) {
    if (payload_string == "1") {
        _bms->restart();
    }
}

void MqttAdapter::on_mqtt_set_config(const String &topic_string, const String &payload_string) {
    int index_of_comma = payload_string.indexOf(",");
    String module_number_string;
    String current_measurement;
    String master_can;

    if (index_of_comma <= 0) {
        module_number_string = payload_string;
    } else {
        int index_of_second_comma = payload_string.indexOf(",", index_of_comma + 1);
        module_number_string = payload_string.substring(0, index_of_comma);

        if(index_of_second_comma > index_of_comma) {
            current_measurement = payload_string.substring(index_of_comma + 1, index_of_second_comma);
            master_can = payload_string.substring(index_of_second_comma + 1);
        }
    }

    if (current_measurement == "1") {
        _current_measurement = true;
    }

    if (master_can == "1") {
        startCANThread();
    }

    if (is_uint(module_number_string)) {
        uint8_t module_number = module_number_string.toInt();
        _bms->set_module_number(module_number);
        _module_topic = "esp-module/" + module_number_string;
        _mqtt.setWill((_module_topic + "/available").c_str(), 0, true, "offline");
        _mqtt.disconnect();
        _mqtt.connect();
    }
}

void MqttAdapter::set_ota_server(String ota_server) {
    _ota_server = ota_server;
}

void MqttAdapter::set_ota_cert(std::string_view cert) {
    _ota_cert = cert;
}

void MqttAdapter::on_mqtt_ota(const String &topic_string, const String &payload_string) {
    if (_ota_cert == nullptr || _ota_server.isEmpty()) {
        publish_value(_mac_topic + "/ota_start", "ota error [no cert or server set]");
        return;
    }

    publish_value(_mac_topic + "/ota_start", "ota started [" + payload_string + "] (" + String(millis()) + ")");
    publish_value(_mac_topic + "/ota_url", "https://" + _ota_server + payload_string);
    String ota_result = WifiManager::perform_ota_update(_ota_server + payload_string, _ota_cert.data());
    publish_value(_mac_topic + "/ota_ret", ota_result);
}