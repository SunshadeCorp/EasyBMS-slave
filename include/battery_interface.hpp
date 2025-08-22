#pragma once

#include <vector>
#include <bitset>

#include "battery_type.hpp"

class BatteryInterface {
   public:
    virtual void init() = 0;
    virtual bool detect_battery() = 0;
    virtual void set_battery_type(BatteryType type) = 0;
    virtual BatteryType battery_type() = 0;
    virtual void set_balance_bits(const std::vector<bool> &balance_bits) = 0;
    virtual std::vector<bool> get_balance_bits() = 0;
    virtual void measure_cells() = 0;
    virtual void measure_aux() = 0;
    virtual std::vector<float> module_temps() = 0;
    virtual std::vector<float> pcb_temps() = 0;
    virtual float chip_temp() = 0;
    virtual float module_voltage() = 0;
    virtual std::vector<float> cell_voltages() = 0;
    virtual bool balance_error() = 0;
    virtual bool measure_error() = 0;
};