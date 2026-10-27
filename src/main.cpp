#include "Arduino.h"
#include "balancer_interface.hpp"
#include "battery_monitor.hpp"
#include "debug.hpp"
#include "display.hpp"
#include "ltc_meb_wrapper.hpp"
#include "mqtt_adapter.hpp"
#include "simulated_battery.hpp"
#include "single_mode_balancer.hpp"
#include "timed_history.hpp"
#include "wifi.hpp"
#include "can_manager.h"
#include "mqtt_manager.h"

#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

std::array<std::shared_ptr<BMS>, LTC_COUNT> bmsArr;
std::array<std::shared_ptr<MqttAdapter>, LTC_COUNT> mqtt_adapterArr;
TaskHandle_t  bmsThreadHandle = nullptr;
TaskHandle_t  mqttThreadHandle = nullptr;
TaskHandle_t  canThreadHandle = nullptr;

// #define MOCK_BATTERY

void bms_loop(void *param);
void mqtt_loop(void *param);
void can_loop(void *param);

[[maybe_unused]] void setup() {
    DEBUG_BEGIN(74880);
    DEBUG_PRINTLN("Init start");

    for (int i = 0; auto &bms : bmsArr) {
#ifdef MOCK_BATTERY
        auto battery_interface = std::make_shared<SimulatedBattery>();
        battery_interface->scenario_everything_ok();
#else
        auto battery_interface = std::make_shared<LtcMebWrapper>(i);

        switch(Config::battery_config)
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
        battery_monitor->set_battery_config(Config::battery_config);
        bms = std::make_shared<BMS>();
        bms->set_mode(Config::bms_mode);
        bms->set_battery_monitor(battery_monitor);

        if constexpr (Config::use_mqtt) {
            mqtt_adapterArr[i] = std::make_shared<MqttAdapter>(
            bms, Config::mqtt_server, Config::mqtt_username, Config::mqtt_password, String("easybms-") + WifiManager::mac_string() + "_" + String(i));

            if (i == 0) {
                mqtt_adapterArr[i]->set_ota_server(Config::ota_server.data());
                mqtt_adapterArr[i]->set_ota_cert(Config::trustRoot.data());
                mqtt_adapterArr[i]->init();
            } else {
                mqtt_adapterArr[i]->init(i);
            }
        }

        switch (Config::bms_mode) {
            case BalanceMode::slave:
                if constexpr (Config::use_mqtt) {
                    bms->set_balancer(mqtt_adapterArr[i]);
                }
                break;
            case BalanceMode::single:
                bms->set_balancer(std::make_shared<SingleModeBalancer>(60 * 1000, 30 * 1000));
                break;
            default:
                break;
        }

        if constexpr (Config::use_display) {
            if (i == 0) {
                auto display = std::make_shared<Display>();
                bms->set_display(display);
                display->init();
            }
        }
        
        if (i == (bmsArr.size() - 1)) {
            bms->set_led(true);
        }

        i++;
    }

    xTaskCreatePinnedToCore(
        bms_loop, "BMSLoop", 16384, nullptr, 1, &bmsThreadHandle, 0);

    if constexpr (Config::use_mqtt) {
        WifiManager::connect(String("easybms-") + WifiManager::mac_string(), Config::ssid.data(), Config::password.data());
        WifiManager::syncTime();

        xTaskCreatePinnedToCore(
            mqtt_loop, "MQTTLoop", 16384, nullptr, 1, &mqttThreadHandle, 0);
    }

    DEBUG_PRINTLN("Init finished");
}

void startCANThread() {
    if (canThreadHandle != nullptr) {
        return;
    }

    if constexpr (!Config::use_mqtt) {
        WifiManager::connect(String("easybms-") + WifiManager::mac_string(), Config::ssid.data(), Config::password.data());
        WifiManager::syncTime();
    }

    xTaskCreatePinnedToCore(
        can_loop, "CANLoop", 16384, nullptr, 1, &canThreadHandle, 0);
}

void bms_loop(void *param) {
    static constexpr time_ms LTC_CHECK_INTERVAL = 1000;
    TickType_t lastWake = pdMS_TO_TICKS(millis());

    for(;;) {
        for (auto &bms : bmsArr)
            if (bms)
                bms->loop();

        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LTC_CHECK_INTERVAL));
    }
}

void mqtt_loop(void *param) {
    static constexpr time_ms MQTT_UPDATE_INTERVAL = 1000;
    TickType_t lastWake = pdMS_TO_TICKS(millis());

    for(;;) {
        for (auto &mqtt_adapter : mqtt_adapterArr)
            if (mqtt_adapter)
                mqtt_adapter->loop();

        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MQTT_UPDATE_INTERVAL));
    }
}

void can_loop(void *param) {
    static constexpr time_ms CAN_CHECK_INTERVAL = 100;
    TickType_t lastWake = pdMS_TO_TICKS(millis());

    for(;;) {
        CanManager::loop();
        MqttManager::loop();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CAN_CHECK_INTERVAL));
    }
}

void loop() {
}
