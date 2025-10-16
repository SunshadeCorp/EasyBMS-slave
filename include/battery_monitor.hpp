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
    BatteryMonitor(std::shared_ptr<BatteryInterface> bat);
    bool initialized() const;
    void set_balance_bits(const std::vector<bool>& balance_bits);
    void measure();
    const std::vector<float>& cell_voltages() const;
    const std::vector<bool>& balance_bits() const;
    void set_battery_config(BatteryConfig config);
    BatteryConfig battery_config() const;
    BatteryType battery_type() const;
    float min_voltage() const;
    float max_voltage() const;
    float avg_voltage() const;
    float cell_diff() const;
    float module_voltage() const;
    float module_temp_1() const;
    float module_temp_2() const;
    float chip_temp() const;
    float soc() const;
    uint32_t measure_error_count() const;
    uint32_t balance_error_count() const;
    bool measure_error() const;
    bool balance_error() const;
    std::optional<float> cell_diff_trend() const;

   private:
    void calc_cell_diff_trend();
    void detect_battery(const std::array<float, 12>& voltages);

    std::shared_ptr<BatteryInterface> _bat;
    BatteryConfig _battery_config;
    BatteryType _battery_type;

    bool _initialized;

    // Store cell diff history with 1h retention and 1 min granularity
    TimedHistory<float> _cell_diff_history = TimedHistory<float>(1000 * 60 * 60, 1000 * 60);
    std::vector<float> _cell_voltages;
    std::vector<float> _cell_diffs;
    std::vector<bool> _balance_bits;
    float _min_voltage;
    float _max_voltage;
    float _avg_voltage;
    float _cell_diff;
    float _module_voltage;
    float _module_temp_1;
    float _module_temp_2;
    float _chip_temp;
    float _soc;
    bool _measure_error;
    bool _balance_error;
    uint32_t _balance_error_count{};
    uint32_t _measure_error_count{};
    std::optional<float> _cell_diff_trend;
};
