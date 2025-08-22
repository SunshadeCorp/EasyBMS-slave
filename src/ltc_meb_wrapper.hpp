#pragma once

#include <LTC68041.h>

#include <array>
#include <bitset>

#include "battery_interface.hpp"

class LtcMebWrapper : public BatteryInterface {
   public:
    LtcMebWrapper();
    void init() override;
    void set_balance_bits(const std::bitset<12>& balance_bits) override;
    float module_temp_1() override;
    float module_temp_2() override;
    float chip_temp() override;
    float module_voltage() override;
    std::array<float, 12> cell_voltages() override;
    bool balance_error() override;
    bool measure_error() override;

   private:
    LTC68041 _ltc = LTC68041(18); // CSLTC
    bool _debug_mode;
    bool _balance_error;
    bool _measure_error;
    float raw_voltage_to_real_module_temp(float raw_voltage);
};