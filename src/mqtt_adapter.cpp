#include <WString.h>

#include <algorithm>
#include <bitset>

#include "bms.hpp"
#include "debug.hpp"
#include "mqtt_client.hpp"
#include "version.h"
#include "wifi.hpp"

MqttAdapter::MqttAdapter(const std::shared_ptr<BMS> &bms, const std::shared_ptr<IMqttClient> &mqtt) :
    _bms(bms),
    _mqtt(mqtt),
    _last_connection{0},
    _hostname(),
    _mac_topic(),
    _module_topic(),
    _last_master_uptime{0},
    _balance_start_time{},
    _balance_duration{},
    _ota_cert{nullptr},
    _ota_server()
{
}

bool MqttAdapter::is_uint(const String& number_string) const {
    return std::all_of(number_string.begin(), number_string.end(), isDigit);
}

int MqttAdapter::cell_id_from_name(const String& cell_name) const {
    if (!is_uint(cell_name)) {
        return -1;
    } else {
        return cell_name.toInt() - 1;
    }
}

String MqttAdapter::cpu_description() const {
    return String(ESP.getChipModel()) + " rev " + ESP.getChipRevision() + " " + ESP.getChipCores() + "x" + ESP.getCpuFreqMHz() + "MHz";
}

String MqttAdapter::flash_description() const {
    return String(ESP.getFlashChipSize() / 1024 / 1024) + " MiB, Mode: " + ESP.getFlashChipMode();
}

void MqttAdapter::init() {
    init("");
}

void MqttAdapter::init(const String &index) {
    if(index == "")
        _mac_topic = String("esp-module/") + mac_string();
    else
        _mac_topic = String("esp-module/") + mac_string() + "_" + index;
        
    _module_topic = _mac_topic;
    _mqtt->set_will(_module_topic + "/available", 0, true, "offline");
}

#define callback(f) static_cast<MqttCallback>([this](const String &v1, const String &v2) { f(v1, v2); })

void MqttAdapter::reconnect() {
    // Loop until we're reconnected
    while (!_mqtt->connected()) {
        DEBUG_PRINT("Attempting MQTT connection...");
        if (_mqtt->connect()) {
            DEBUG_PRINTLN("connected");
            // Once connected, publish an announcement
            _mqtt->publish(_module_topic + "/available", "online");
            if (!_module_topic.equals(_mac_topic)) {
                _mqtt->publish(_mac_topic + "/available", "undefined");
            }
            _mqtt->publish(_mac_topic + "/module_topic", _module_topic);
            _mqtt->publish(_mac_topic + "/version", VERSION);
            _mqtt->publish(_mac_topic + "/build_timestamp", BUILD_TIMESTAMP);
            _mqtt->publish(_mac_topic + "/wifi", WiFi.SSID());
            _mqtt->publish(_mac_topic + "/ip", WiFi.localIP().toString());
            _mqtt->publish(_mac_topic + "/esp_sdk", ESP.getSdkVersion());
            _mqtt->publish(_mac_topic + "/cpu", cpu_description());
            _mqtt->publish(_mac_topic + "/flash", flash_description());
            _mqtt->publish(_mac_topic + "/bms_mode", as_string(_bms->mode()));
            // Resubscribe
            _mqtt->subscribe("master/uptime", callback(on_mqtt_master_uptime));
            for (int i = 0; i < 12; ++i) {
                _mqtt->subscribe(_module_topic + "/cell/" + (i + 1) + "/balance_request", callback(on_mqtt_balance_request));
            }
            _mqtt->subscribe(_mac_topic + "/blink", callback(on_mqtt_blink));
            _mqtt->subscribe(_mac_topic + "/set_config", callback(on_mqtt_set_config));
            _mqtt->subscribe(_mac_topic + "/restart", callback(on_mqtt_restart));
            _mqtt->subscribe(_mac_topic + "/ota", callback(on_mqtt_ota));
        } else {
            DEBUG_PRINT("failed, rc=");
            DEBUG_PRINTLN(_mqtt->state_string());
            if (_last_connection != 0 && millis() - _last_connection >= 30000) {
                _bms->restart();
            }
            delay(1000);
        }
    }
}

void MqttAdapter::loop() {
    if (!_mqtt->connected()) {
        reconnect();
    }

    _bms->loop();
    update();

    _last_connection = millis();
    _mqtt->loop();
}

void MqttAdapter::reset_balancing(size_t size) {
    _balance_start_time.assign(size, false);
    _balance_duration.assign(size, false);
}

void MqttAdapter::balance(const std::vector<float>& voltages) {
    // Do nothing
}

std::vector<bool> MqttAdapter::balance_bits() {
    auto number_of_cells = _bms->battery_monitor()->cell_voltages().size();
    auto balance_bits = std::vector<bool>(number_of_cells, false);

    if (_balance_start_time.size() != number_of_cells) {
        reset_balancing(balance_bits.size());
        return balance_bits;
    }

    for (size_t i = 0; i < _balance_start_time.size(); ++i) {
        if (millis() - _balance_start_time[i] <= _balance_duration[i]) {
            balance_bits[i] = true;
        } else if (_balance_duration[i] > 0) {
            _balance_duration[i] = 0;
        }
    }

    return balance_bits;
}

void MqttAdapter::publish(const String &topic) {
    auto m = _bms->battery_monitor();
    auto balance_bits = m->balance_bits();
    String tmp = String();

    _mqtt->publish(topic + "/uptime", millis());
    _mqtt->publish(topic + "/pec15_error_count", m->measure_error_count());
    _mqtt->publish(topic + "/battery_config", as_string(m->battery_config()));

    // Don't send measurements if invalid
    if (m->measure_error()) {
        return;
    }

    for (size_t i = 0; const auto &v : m->cell_voltages()) {
        String cell_name = String(i + 1);
        if (cell_name != "undefined") {
            _mqtt->publish(topic + "/cell/" + cell_name + "/voltage", String(v, 3));
            _mqtt->publish(topic + "/cell/" + cell_name + "/is_balancing", balance_bits[i++] ? "1" : "0");
        }
    }

    _mqtt->publish(topic + "/module_voltage", m->module_voltage());

    for(const auto &temp : m->module_temps()) {
        tmp += String(temp) + ",";
    }

    _mqtt->publish(topic + "/module_temps", tmp);

    for(const auto &temp : m->pcb_temps()) {
        tmp += String(temp) + ",";
    }

    _mqtt->publish(topic + "/pcb_temps", tmp);
    _mqtt->publish(topic + "/chip_temp", m->chip_temp());
    _mqtt->publish(topic + "/battery_type", as_string(m->battery_type()));
    _mqtt->publish(topic + "/battery_current", m->battery_current());
}

void MqttAdapter::update() {
    bool is_balancing = false;

    auto balance_bits = _bms->battery_monitor()->balance_bits();
    for (size_t i = 0; i < balance_bits.size(); i++) {
        if (balance_bits[i]) {
            is_balancing = true;
        }
    }

    if (!is_balancing) {
        // Deprecated
        publish(_module_topic + "/accurate");
    }

    publish(_module_topic);
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
    String cell_name = topic_string.substring((_module_topic + "/cell/").length());
    cell_name = cell_name.substring(0, cell_name.indexOf("/"));
    int cell_id = cell_id_from_name(cell_name);
    if (cell_id == -1 || static_cast<size_t>(cell_id) >= _balance_duration.size()) {
        DEBUG_PRINTLN("MQTT: Got invalid cell name for balancing: " + cell_name);
        DEBUG_PRINTLN("_balance_duration.size(): " + String(_balance_duration.size()));
        DEBUG_PRINTLN("_balance_start_time.size(): " + String(_balance_start_time.size()));
        return;
    }

    time_ms balance_time = payload_string.toInt();

    if (balance_time > 1000 * 60 * 5) {
        DEBUG_PRINTLN("MQTT: Balance time too long (> 5 mins): " + String(balance_time) + "ms");
        return;
    }

    _balance_start_time[cell_id] = millis();
    _balance_duration[cell_id] = balance_time;
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
    int indexOfComma = payload_string.indexOf(",");
    String module_number_string;
    if (indexOfComma >= 0) {
        module_number_string = payload_string.substring(0, indexOfComma);
    } else {
        module_number_string = payload_string;
    }

    if (is_uint(module_number_string)) {
        uint8_t module_number = module_number_string.toInt();
        _bms->set_module_number(module_number);
        _module_topic = "esp-module/" + module_number_string;
        _mqtt->disconnect();
        reconnect();
    }
}

void MqttAdapter::set_ota_server(String ota_server) {
    _ota_server = ota_server;
}

void MqttAdapter::set_ota_cert(const char* cert) {
    _ota_cert = cert;
}

void MqttAdapter::on_mqtt_ota(const String &topic_string, const String &payload_string) {
    if((_ota_cert == nullptr) || _ota_server.isEmpty()) {
        _mqtt->publish(_mac_topic + "/ota_start", "ota error [no cert or server set]");
        return;
    } else {
        _mqtt->publish(_mac_topic + "/ota_start", "ota started [" + payload_string + "] (" + String(millis()) + ")");
    }
    
    _mqtt->publish(_mac_topic + "/ota_url", "https://" + _ota_server + payload_string);
    String ota_result = perform_ota_update(_ota_server + payload_string, _ota_cert);
    _mqtt->publish(_mac_topic + "/ota_ret", ota_result);
}