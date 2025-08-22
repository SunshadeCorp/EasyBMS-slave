#pragma once

#include <array>
#include <bitset>
#include <memory>
#include <optional>
#include <vector>

#include "battery_interface.hpp"
#include "battery_type.hpp"
#include "timed_history.hpp"

class BatteryMonitor {
   public:
    BatteryMonitor(const std::shared_ptr<BatteryInterface> &bat);
    void set_balance_bits(const std::vector<bool>& balance_bits);
    void measure();
    void calc_cell_voltages();
    void calc_aux_data();
    const std::vector<float>& cell_voltages() const;
    std::vector<bool> balance_bits() const;
    void set_battery_config(BatteryConfig config);
    BatteryConfig battery_config() const;
    BatteryType battery_type() const;
    float min_voltage() const;
    float max_voltage() const;
    float avg_voltage() const;
    float cell_diff() const;
    float module_voltage() const;
    const std::vector<float>& module_temps() const;
    const std::vector<float>& pcb_temps() const;
    float chip_temp() const;
    float soc() const;
    uint32_t measure_error_count() const;
    uint32_t balance_error_count() const;
    bool measure_error() const;
    bool balance_error() const;
    std::optional<float> cell_diff_trend() const;

   private:
    void calc_cell_diff_trend();

    std::shared_ptr<BatteryInterface> _bat;
    BatteryConfig _battery_config;

    // Store cell diff history with 1h retention and 1 min granularity
    TimedHistory<float> _cell_diff_history;
    std::vector<float> _cell_voltages;
    std::vector<float> _cell_diffs;
    std::vector<float> _module_temps;
    std::vector<float> _pcb_temps;
    float _min_voltage;
    float _max_voltage;
    float _avg_voltage;
    float _cell_diff;
    float _module_voltage;
    float _chip_temp;
    float _soc;
    bool _measure_error;
    bool _balance_error;
    uint32_t _balance_error_count;
    uint32_t _measure_error_count;
    std::optional<float> _cell_diff_trend;
};
