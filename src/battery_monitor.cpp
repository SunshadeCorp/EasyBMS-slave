#include "battery_monitor.hpp"

#include <algorithm>
#include <numeric>

#include "battery_type.hpp"
#include "soc.hpp"

#define SECONDS 1000
#define MINUTES (60 * SECONDS)
#define HOURS (60 * MINUTES)

BatteryMonitor::BatteryMonitor(std::shared_ptr<BatteryInterface> bat) {
    _cell_diff_trend = {};
    _bat = bat;
    _bat->init();
}

void BatteryMonitor::set_battery_config(BatteryConfig config) {
    _battery_config = config;
}

const std::vector<float>& BatteryMonitor::cell_voltages() const {
    return _cell_voltages;
}

const std::vector<bool>& BatteryMonitor::balance_bits() const {
    return _balance_bits;
}

BatteryType BatteryMonitor::battery_type() const {
    return _battery_type;
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
    _balance_bits = balance_bits;

    std::bitset<12> bits;
    if (_battery_type == BatteryType::meb8s) {
        bits[0] = balance_bits[0];
        bits[1] = balance_bits[1];
        bits[2] = balance_bits[2];
        bits[3] = balance_bits[3];

        bits[4] = false;
        bits[5] = false;
        bits[6] = false;
        bits[7] = false;

        bits[8] = balance_bits[4];
        bits[9] = balance_bits[5];
        bits[10] = balance_bits[6];
        bits[11] = balance_bits[7];
    } else {
        for (size_t i = 0; i < bits.size(); i++) {
            bits[i] = balance_bits[i];
        }
    }

    _bat->set_balance_bits(bits);
}

void BatteryMonitor::detect_battery(const std::array<float, 12>& voltages) {
    if (_battery_config == BatteryConfig::meb12s) {
        _battery_type = BatteryType::meb12s;
    } else if (_battery_config == BatteryConfig::meb8s) {
        _battery_type = BatteryType::meb8s;
    } else {
        _battery_type = detect_battery_type(voltages);
    }
}

void BatteryMonitor::measure() {
    auto ltc_voltages = _bat->cell_voltages();
    detect_battery(ltc_voltages);

    if (_battery_type == BatteryType::meb8s) {
        _cell_voltages.resize(8);
        _cell_voltages[0] = ltc_voltages[0];
        _cell_voltages[1] = ltc_voltages[1];
        _cell_voltages[2] = ltc_voltages[2];
        _cell_voltages[3] = ltc_voltages[3];
        _cell_voltages[4] = ltc_voltages[8];
        _cell_voltages[5] = ltc_voltages[9];
        _cell_voltages[6] = ltc_voltages[10];
        _cell_voltages[7] = ltc_voltages[11];
    } else {
        _cell_voltages.resize(12);
        for (size_t i = 0; i < 12; i++) {
            _cell_voltages[i] = ltc_voltages[i];
        }
    }

    if (_cell_voltages.size() != _balance_bits.size()) {
        _balance_bits.assign(_cell_voltages.size(), false);
    }

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
    _module_voltage = _bat->module_voltage();
    _module_temp_1 = _bat->module_temp_1();
    _module_temp_2 = _bat->module_temp_2();
    _chip_temp = _bat->chip_temp();
    _soc = SOC::voltage_to_soc(_avg_voltage);
    calc_cell_diff_trend();
    _balance_error = _bat->balance_error();
    if (_balance_error) {
        _balance_error_count++;
    }
    _measure_error = _bat->measure_error();
    if (_measure_error) {
        _measure_error_count++;
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

float BatteryMonitor::module_temp_1() const {
    return _module_temp_1;
}

float BatteryMonitor::module_temp_2() const {
    return _module_temp_2;
}

float BatteryMonitor::chip_temp() const {
    return _chip_temp;
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