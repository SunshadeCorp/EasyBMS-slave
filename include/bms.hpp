#pragma once

#include "balance_mode.hpp"
#include "balancer_interface.hpp"
#include "battery_monitor.hpp"
#include "display.hpp"
#include "mqtt_adapter.hpp"

#include <atomic>


class MqttAdapter;

using time_ms = unsigned long;

class BMS {
   public:
    BMS();
    void blink();
    void flip_led();
    void set_led(bool led_state);
    void restart();
    void set_module_number(uint8_t module_number);
    void set_mode(BalanceMode mode);
    void set_balancer(const std::shared_ptr<IBalancer> &balancer);
    void set_display(const std::shared_ptr<Display> &display);
    void set_mqtt_adapter(const std::shared_ptr<MqttAdapter> &mqtt_adapter);
    void set_battery_monitor(const std::shared_ptr<BatteryMonitor> &battery_monitor);
    BalanceMode mode() const;
    std::shared_ptr<const BatteryMonitor> battery_monitor();
    void loop();

   private:
    static constexpr time_ms BLINK_TIME = 5000;
    std::atomic<time_ms> _last_blink_time;
    uint8_t _module_number;
    BalanceMode _mode;
    std::shared_ptr<IBalancer> _balancer;
    std::shared_ptr<BatteryMonitor> _battery_monitor;
    std::shared_ptr<Display> _display;
    bool _led_builtin_state;
};