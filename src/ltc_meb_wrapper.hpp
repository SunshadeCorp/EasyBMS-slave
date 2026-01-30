#pragma once

#include <LTC68041.h>

#include <vector>
#include <bitset>
#include <atomic>

#include "battery_interface.hpp"

#include "config.h"

class LtcMebWrapper : public BatteryInterface {
   public:
    LtcMebWrapper(size_t index = 0);
    void init() override;
    bool detect_battery() override;
    void set_battery_type(BatteryType type) override;
    BatteryType battery_type() override;
    void set_balance_bits(const std::vector<bool> &balance_bits) override;
    std::vector<bool> get_balance_bits() override;
    bool is_balancing() override;
    void measure_cells() override;
    void measure_temps() override;
    void measure_aux() override;
    std::vector<float> module_temps() override;
    std::vector<float> pcb_temps() override;
    float chip_temp() override;
    float module_voltage() override;
    std::vector<float> cell_voltages() override;
    float aux_voltage() override;
    bool balance_error() override;
    bool measure_error() override;

   private:
    template<std::size_t N>
    std::vector<float> get_cells() {
        std::array<float, N> voltages;
        bool success;

        if constexpr (ltc_count > 1) {
            switch (_ltc_index) {
                case 0:
                success = _ltc.getCellVoltages<N,0>(voltages);
                break;
                case 1:
                success = _ltc.getCellVoltages<N,1>(voltages);
                break;
                default:
                break;
            }
        } else {
            success = _ltc.getCellVoltages(voltages);
        }

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
    static std::atomic<bool> initialized;
    static LTC68041<ltc_count> _ltc;
    const size_t _ltc_index;
    bool _balance_error;
    bool _measure_error;
    bool _balancing;
    constexpr float raw_voltage_to_real_temp(float raw_voltage, float r2, float r0, int beta);
};