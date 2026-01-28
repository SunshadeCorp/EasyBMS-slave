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

std::array<std::shared_ptr<BMS>, ltc_count> bmsArr;
std::array<std::shared_ptr<MqttAdapter>, ltc_count> mqtt_adapterArr;

// #define MOCK_BATTERY
// #define MOCK_MQTT

[[maybe_unused]] void setup() {
    DEBUG_BEGIN(74880);
    DEBUG_PRINTLN("init start");

    auto hostname = String("easybms-") + mac_string();

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

    if constexpr (use_mqtt) {
        DEBUG_PRINTLN("Setup MQTT");
        connect_wifi(hostname, ssid, password);
    }

    for (int i = 0; auto &bms : bmsArr) {
    #ifdef MOCK_BATTERY
    auto battery_interface = std::make_shared<SimulatedBattery>();
    battery_interface->scenario_everything_ok();
    #else
        auto battery_interface = std::make_shared<LtcMebWrapper>(i);

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
#endif

        auto battery_monitor = std::make_shared<BatteryMonitor>(battery_interface);
        battery_monitor->set_battery_config(battery_config);
        bms = std::make_shared<BMS>();
        bms->set_mode(bms_mode);
        bms->set_battery_monitor(battery_monitor);

        if constexpr (use_mqtt) {
            mqtt_adapterArr[i] = std::make_shared<MqttAdapter>(bms, mqtt);

            if (i == 0) {
                mqtt_adapterArr[i]->set_ota_server(ota_server);
                mqtt_adapterArr[i]->set_ota_cert(trustRoot);
                mqtt_adapterArr[i]->init();
            } else {
                mqtt_adapterArr[i]->init(String(i));
            }
        }

        switch (bms_mode) {
            case BalanceMode::slave:
                if constexpr (use_mqtt) {
                    bms->set_balancer(mqtt_adapterArr[i]);
                }
                break;
            case BalanceMode::single:
                bms->set_balancer(std::make_shared<SingleModeBalancer>(60 * 1000, 30 * 1000));
                break;
            default:
                break;
        }

        if (i == 0) {
            auto display = std::make_shared<Display>();
            bms->set_display(display);
            display->init();
        }
        
        if (i == (bmsArr.size() - 1)) {
            bms->set_led(true);
        }

        i++;
    }
    }

    DEBUG_PRINTLN("init finished");
}

void loop() {
    if constexpr (use_mqtt) {
        for (auto &mqtt_adapter : mqtt_adapterArr)
            if (mqtt_adapter)
                mqtt_adapter->loop();
    } else {
        for (auto &bms : bmsArr)
            if (bms)
                bms->loop();
    }

    delay(100);
}
