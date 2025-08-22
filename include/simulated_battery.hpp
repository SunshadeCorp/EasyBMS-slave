#pragma once

#include <array>
#include <bitset>

#include "battery_interface.hpp"

class SimulatedBattery : public BatteryInterface {
   public:
    SimulatedBattery();
    void init() override;
    void set_balance_bits(const std::vector<bool>& balance_bits) override;
    std::vector<bool> get_balance_bits() override;
    std::vector<float> module_temps() override;
    std::vector<float> pcb_temps() override;
    float chip_temp() override;
    float module_voltage() override;
    std::vector<float> cell_voltages() override;
    bool balance_error() override;
    bool measure_error() override;

    void scenario_everything_ok();
    void scenario_balance();
    void scenario_measure_error();
    void scenario_8s();
    void scenario_random();

    void balance();
    std::vector<float> wiggle(std::vector<float> voltages);

   private:
    std::vector<float> _voltages;
    std::vector<bool> _balance_bits;
    bool _measure_error = false;
    bool _balance_error = false;
};