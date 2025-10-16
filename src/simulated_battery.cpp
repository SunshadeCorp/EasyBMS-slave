#include "simulated_battery.hpp"
#include <esp32-hal.h>

SimulatedBattery::SimulatedBattery() {
    // Do nothing
}

void SimulatedBattery::init() {
    // Do nothing
}

void SimulatedBattery::set_balance_bits(const std::bitset<12>& balance_bits) {
    _balance_bits = balance_bits;
}

void SimulatedBattery::scenario_everything_ok() {
    _voltages[0] = 3.7;
    _voltages[1] = 3.7;
    _voltages[2] = 3.7;
    _voltages[3] = 3.7;
    _voltages[4] = 3.7;
    _voltages[5] = 3.7;
    _voltages[6] = 3.7;
    _voltages[7] = 3.7;
    _voltages[8] = 3.7;
    _voltages[9] = 3.7;
    _voltages[10] = 3.7;
    _voltages[11] = 3.7;
}

void SimulatedBattery::scenario_balance() {
    _voltages[0] = 3.7;
    _voltages[1] = 3.7;
    _voltages[2] = 3.7;
    _voltages[3] = 3.9;
    _voltages[4] = 3.7;
    _voltages[5] = 3.7;
    _voltages[6] = 3.7;
    _voltages[7] = 3.7;
    _voltages[8] = 3.7;
    _voltages[9] = 3.7;
    _voltages[10] = 3.7;
    _voltages[11] = 3.7;
}

void SimulatedBattery::scenario_8s() {
    _voltages[0] = 3.7;
    _voltages[1] = 3.7;
    _voltages[2] = 3.7;
    _voltages[3] = 3.9;
    _voltages[4] = 0.0;
    _voltages[5] = 0.0;
    _voltages[6] = 0.0;
    _voltages[7] = 0.0;
    _voltages[8] = 3.7;
    _voltages[9] = 3.7;
    _voltages[10] = 3.7;
    _voltages[11] = 3.7;
}

void SimulatedBattery::scenario_random() {
    _voltages[0] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[1] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[2] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[3] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[4] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[5] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[6] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[7] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[8] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[9] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[10] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    _voltages[11] = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
}

void SimulatedBattery::scenario_measure_error() {
    _measure_error = true;
}

float SimulatedBattery::module_temp_1() {
    return 20;
}
float SimulatedBattery::module_temp_2() {
    return 20;
}
float SimulatedBattery::chip_temp() {
    return 22;
}
float SimulatedBattery::module_voltage() {
    float sum = 0;
    for (size_t i = 0; i < 12; i++) {
        sum += _voltages[i];
    }
    return sum;
}

void SimulatedBattery::balance() {
    for (size_t i = 0; i < 12; i++) {
        if (_balance_bits[i]) {
            _voltages[i] = _voltages[i] * 0.9998;
        }
    }
}

std::array<float, 12> SimulatedBattery::wiggle(std::array<float, 12> voltages) {
    float wiggle_room = 0.0025;
    for (size_t i = 0; i < 12; i++) {
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        if (voltages[i] > 0.1) {
            voltages[i] += (r - 0.5) * wiggle_room;
        }
    }

    return voltages;
}

std::array<float, 12> SimulatedBattery::cell_voltages() {
    delay(2000);
    balance();
    return wiggle(_voltages);
}

bool SimulatedBattery::balance_error() {
    return _balance_error;
}
bool SimulatedBattery::measure_error() {
    return _measure_error;
}