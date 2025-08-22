#pragma once

#include <LTC68041.h>

#include <vector>
#include <bitset>

#include "battery_interface.hpp"

class LtcMebWrapper : public BatteryInterface {
   public:
    LtcMebWrapper();
    void init() override;
    bool detect_battery() override;
    void set_battery_type(BatteryType type) override;
    BatteryType battery_type() override;
    void set_balance_bits(const std::vector<bool> &balance_bits) override;
    std::vector<bool> get_balance_bits() override;
    void measure_cells() override;
    void measure_aux() override;
    std::vector<float> module_temps() override;
    std::vector<float> pcb_temps() override;
    float chip_temp() override;
    float module_voltage() override;
    std::vector<float> cell_voltages() override;
    bool balance_error() override;
    bool measure_error() override;

   private:
    template<std::size_t N>
    std::vector<float> get_cells() {
        std::array<float, N> voltages;
        bool success = _ltc.getCellVoltages(voltages);

        if (success) {
            _measure_error = false;
        } else {
            _measure_error = true;
            // for (auto& voltage : voltages) {
            //     voltage = -1.f;
            // }
        }

        return std::vector<float>(voltages.begin(), voltages.end());
    }

    BatteryType _bat_type;
    LTC68041 _ltc;
    bool _balance_error;
    bool _measure_error;
    float raw_voltage_to_real_module_temp(float raw_voltage);
};