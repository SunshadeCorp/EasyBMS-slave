#include "simulated_battery.hpp"

SimulatedBattery::SimulatedBattery() : _voltages(12), _balance_bits(12)
{
    // Do nothing
}

void SimulatedBattery::init() {
    // Do nothing
}

void SimulatedBattery::set_balance_bits(const std::vector<bool>& balance_bits) {
    _balance_bits = balance_bits;
}

void SimulatedBattery::scenario_everything_ok() {
    for(auto &volt : _voltages)
        volt = 3.7;
}

void SimulatedBattery::scenario_balance() {
    for(int i = 0; i < 3; i++)
        _voltages[i] = 3.7;

    _voltages[3] = 3.9;

    for(int i = 4; i < _voltages.size(); i++)
        _voltages[i] = 3.7;
}

void SimulatedBattery::scenario_8s() {
    _voltages.resize(8);

    scenario_balance();
}

void SimulatedBattery::scenario_random() {
    for(auto &volt : _voltages)
        volt = (3.5) + 0.3 * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
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
    for (auto &volt : _voltages) {
        sum += volt;
    }
    return sum;
}

void SimulatedBattery::balance() {
    if((_balance_bits.size() != 12) || (_balance_bits.size() != 8))
        return;

    for (int i = 0; auto &volt : _voltages) {           
        if (_balance_bits[i++])
            volt *= 0.9998;

        if(_balance_bits.size() == 12) {
            if(i == 4)
                i += 4;
        }
    }
}

std::vector<float> SimulatedBattery::wiggle(std::vector<float> voltages) {
    float wiggle_room = 0.0025;
    for (auto &volt : voltages) {
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        if (volt > 0.1) {
            volt += (r - 0.5) * wiggle_room;
        }
    }

    return voltages;
}

std::vector<float> SimulatedBattery::cell_voltages() {
    balance();
    return wiggle(_voltages);
}

bool SimulatedBattery::balance_error() {
    return _balance_error;
}
bool SimulatedBattery::measure_error() {
    return _measure_error;
}