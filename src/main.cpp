#include "Arduino.h"
#include "balancer_interface.hpp"
#include "battery_monitor.hpp"
#include "config.h"
#include "debug.hpp"
#include "display.hpp"
#include "ltc_meb_wrapper.hpp"
#include "mock_mqtt_client.hpp"
#include "mqtt_adapter.hpp"
#include "mqtt_client.hpp"
#include "simulated_battery.hpp"
#include "single_mode_balancer.hpp"
#include "timed_history.hpp"
#include "wifi.hpp"

std::shared_ptr<BMS> bms;
std::shared_ptr<MqttAdapter> mqtt_adapter;

// #define MOCK_BATTERY
// #define MOCK_MQTT

[[maybe_unused]] void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(6, OUTPUT); // SCLK
    digitalWrite(LED_BUILTIN, false);

    DEBUG_BEGIN(74880);
    DEBUG_PRINTLN("init");

    #ifdef MOCK_BATTERY
    auto battery_interface = std::make_shared<SimulatedBattery>();
    battery_interface->scenario_everything_ok();
    #else
    auto battery_interface = std::make_shared<LtcMebWrapper>();

    switch(battery_config)
    {
        case BatteryConfig::meb12s:
            battery_interface->set_battery_type(BatteryType::meb12s);
            break;
        case BatteryConfig::meb8s:
            battery_interface->set_battery_type(BatteryType::meb8s);
            break;
        case BatteryConfig::mebAuto:
            if(!battery_interface->detect_battery())
                battery_interface->set_battery_type(BatteryType::meb12s);
            break;
        default:
            break;
    }

    auto battery_monitor = std::make_shared<BatteryMonitor>(battery_interface);
    battery_monitor->set_battery_config(battery_config);
    auto display = std::make_shared<Display>();
    bms = std::make_shared<BMS>();
    bms->set_mode(bms_mode);
    bms->set_display(display);
    bms->set_battery_monitor(battery_monitor);

    display->init();

    if (use_mqtt) {
        DEBUG_PRINTLN("Setup MQTT");
        auto hostname = String("easybms-") + mac_string();
        connect_wifi(hostname, ssid, password);
        digitalWrite(LED_BUILTIN, true);

        #ifdef MOCK_MQTT
        auto mqtt = std::make_shared<MockMqttClient>();
        mqtt->is_connected = false;
        mqtt->connect_result = true;
        #else
        auto mqtt = std::make_shared<MqttClient>(mqtt_server, mqtt_port);
        mqtt->set_user(mqtt_username);
        mqtt->set_password(mqtt_password);
        mqtt->set_id(hostname);
        #endif

        mqtt_adapter = std::make_shared<MqttAdapter>(bms, mqtt);
        mqtt_adapter->set_ota_server(ota_server);
        mqtt_adapter->set_ota_cert(trustRoot);
        mqtt_adapter->init();
    }

    if (bms_mode == BalanceMode::single) {
        bms->set_balancer(std::make_shared<SingleModeBalancer>(60 * 1000, 30 * 1000));
    } else if (bms_mode == BalanceMode::slave && use_mqtt) {
        bms->set_balancer(mqtt_adapter);
    }
}

void loop() {
    if (mqtt_adapter) {
        mqtt_adapter->loop();
    } else {
        bms->loop();
    }
}
