#include "single_mode_balancer.hpp"

#include <Arduino.h>

#include <vector>

void SingleModeBalancer::reset_balance_bits() {
    for (size_t i = 0; i < _balance_bits.size(); i++) {
        _balance_bits[i] = false;
    }
}

void SingleModeBalancer::select_cells_to_balance(const std::vector<float>& voltages) {
    float target = min_voltage(voltages);

    if (target <= _cut_off_voltage) {
        // Don't balance
        reset_balance_bits();
    } else {
        // Balance all the cells above target_voltage
        for (size_t i = 0; i < voltages.size(); i++) {
            if (voltages[i] > target + 0.005) {
                _balance_bits[i] = true;
            } else {
                _balance_bits[i] = false;
            }
        }
    }
}

float SingleModeBalancer::min_voltage(const std::vector<float>& voltages) const {
    float min = voltages[0];

    for (size_t i = 0; i < voltages.size(); i++) {
        if (voltages[i] < min) {
            min = voltages[i];
        }
    }

    return min;
}

SingleModeBalancer::SingleModeBalancer(long balance_time_ms, long relax_time_ms) :
    _balance_time_ms{balance_time_ms},
    _relax_time_ms{relax_time_ms},
    _balance_start_timestamp{0},
    _relax_start_timestamp{0},
    _cut_off_voltage{3.5},
    _balancer_state(BalancerState::Idle),
    _balance_bits{}
{
}

std::vector<bool> SingleModeBalancer::balance(const std::vector<float>& voltages) {
    if (_balance_bits.size() != voltages.size()) {
        _balance_bits.resize(voltages.size());
        reset_balance_bits();
        _balancer_state = BalancerState::Idle;
    }

    long time = millis();

    if (_balancer_state == BalancerState::Relaxing) {
        if (time > _relax_start_timestamp + _relax_time_ms) {
            _balancer_state = BalancerState::Idle;
        }
    }

    if (_balancer_state == BalancerState::Balancing) {
        if (time > _balance_start_timestamp + _balance_time_ms) {
            reset_balance_bits();
            _relax_start_timestamp = time;
            _balancer_state = BalancerState::Relaxing;
        }
    }

    if (_balancer_state == BalancerState::Idle) {
        select_cells_to_balance(voltages);
        _balance_start_timestamp = time;
        _balancer_state = BalancerState::Balancing;
    }

    return _balance_bits;
}