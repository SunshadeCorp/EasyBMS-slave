#include "battery_monitor.hpp"

#include <algorithm>
#include <numeric>

#include "battery_type.hpp"
#include "soc.hpp"

#define SECONDS 1000
#define MINUTES (60 * SECONDS)
#define HOURS (60 * MINUTES)

BatteryMonitor::BatteryMonitor(const std::shared_ptr<BatteryInterface> &bat) :
    _bat(bat),
    _battery_config(BatteryConfig::mebAuto),
    _cell_diff_history(1000 * 60 * 60, 1000 * 60),
    _cell_voltages{},
    _cell_diffs{},
    _module_temps{},
    _pcb_temps{},
    _min_voltage{},
    _max_voltage{},
    _avg_voltage{},
    _cell_diff{},
    _module_voltage{},
    _chip_temp{},
    _soc{},
    _measure_error{},
    _balance_error{},
    _balance_error_count{},
    _measure_error_count{},
    _cell_diff_trend{}
{
    _bat->init();
}

void BatteryMonitor::set_battery_config(BatteryConfig config) {
    _battery_config = config;
}

void BatteryMonitor::calc_cell_voltages() {
    _bat->measure_cells();
    _cell_voltages = _bat->cell_voltages();
    _module_voltage = _bat->module_voltage();

    _min_voltage = *std::min_element(_cell_voltages.begin(), _cell_voltages.end());
    _max_voltage = *std::max_element(_cell_voltages.begin(), _cell_voltages.end());
    _cell_diff = _max_voltage - _min_voltage;
    _cell_diff_history.insert(_cell_diff);
    float voltages_sum = 0.0f;
    for (size_t i = 0; i < _cell_voltages.size(); i++) {
        voltages_sum += _cell_voltages[i];
    }
    _avg_voltage = voltages_sum / static_cast<float>(_cell_voltages.size());
    _cell_diffs.resize(_cell_voltages.size());
    for (size_t i = 0; i < _cell_voltages.size(); i++) {
        _cell_diffs[i] = _cell_voltages[i] - _avg_voltage;
    }
    
    _soc = SOC::voltage_to_soc(_avg_voltage);
    calc_cell_diff_trend();
    
    _measure_error = _bat->measure_error();
    if (_measure_error) {
        _measure_error_count++;
    }
}
void BatteryMonitor::calc_temps() {
    _bat->measure_temps();
    _chip_temp = _bat->chip_temp();
    _module_temps = _bat->module_temps();
    _pcb_temps = _bat->pcb_temps();
}

const std::vector<float>& BatteryMonitor::cell_voltages() const {
    return _cell_voltages;
}

std::vector<bool> BatteryMonitor::balance_bits() const {
    return _bat->get_balance_bits();
}

bool BatteryMonitor::is_balancing() const {
    return _bat->is_balancing();
}

BatteryType BatteryMonitor::battery_type() const {
    return _bat->battery_type();
}

BatteryConfig BatteryMonitor::battery_config() const {
    return _battery_config;
}

bool BatteryMonitor::measure_error() const {
    return _measure_error;
}
bool BatteryMonitor::balance_error() const {
    return _balance_error;
}

void BatteryMonitor::set_balance_bits(const std::vector<bool>& balance_bits) {
    _bat->set_balance_bits(balance_bits);
    _balance_error = _bat->balance_error();
    if (_balance_error) {
        _balance_error_count++;
    }
}

uint32_t BatteryMonitor::measure_error_count() const {
    return _measure_error_count;
}
uint32_t BatteryMonitor::balance_error_count() const {
    return _balance_error_count;
}

float BatteryMonitor::min_voltage() const {
    return _min_voltage;
}

float BatteryMonitor::max_voltage() const {
    return _max_voltage;
}

float BatteryMonitor::avg_voltage() const {
    return _avg_voltage;
}

float BatteryMonitor::cell_diff() const {
    return _cell_diff;
}

float BatteryMonitor::module_voltage() const {
    return _module_voltage;
}

const std::vector<float>& BatteryMonitor::module_temps() const {
    return _module_temps;
}

const std::vector<float>& BatteryMonitor::pcb_temps() const {
    return _pcb_temps;
}

float BatteryMonitor::chip_temp() const {
    return _chip_temp;
}

float BatteryMonitor::battery_current() const {
    _bat->measure_aux();
    return (_bat->aux_voltage() - 2.5f) / 0.02f;
}

float BatteryMonitor::soc() const {
    return _soc;
}

std::optional<float> BatteryMonitor::cell_diff_trend() const {
    return _cell_diff_trend;
}

void BatteryMonitor::calc_cell_diff_trend() {
    auto result_avg = _cell_diff_history.avg_element();
    auto result_latest = _cell_diff_history.newest_element();
    if (result_avg.has_value() && result_latest.has_value()) {
        float avg_cell_diff = result_avg.value().value;
        unsigned long avg_timestamp = result_avg.value().timestamp_ms;
        float latest_cell_diff = result_latest.value().value;
        unsigned long latest_timestamp = result_latest.value().timestamp_ms;

        if (latest_timestamp - avg_timestamp > 2 * MINUTES) {
            // Cell diff change per hour in the last hour
            float change = latest_cell_diff - avg_cell_diff;
            unsigned long time_ms = latest_timestamp - avg_timestamp;
            float time_h = static_cast<float>(time_ms) / static_cast<float>(1 * HOURS);
            _cell_diff_trend = change / time_h;
        }
    }
}