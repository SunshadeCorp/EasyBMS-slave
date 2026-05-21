#include "ltc_meb_wrapper.hpp"

#include <array>

LTC68041<ltc_count> LtcMebWrapper::_ltc(18); // CSLTC
std::atomic<bool> LtcMebWrapper::initialized = false;

LtcMebWrapper::LtcMebWrapper(size_t index) : _ltc_index{index}, _balance_error{false}, _measure_error{false}, _balancing{false}
{
}

void LtcMebWrapper::init() {
    if(initialized)
        return;

    _ltc.initSPI(2, 7, 6); // MOSI, MISO, SCLK

    if (!_ltc.checkSPI()) {
        return;
    }

    initialized = true;

    _ltc.cfgSetRefOn(true);
    _ltc.cfgSetVUV(3.1);
    _ltc.cfgSetVOV(4.2);
    _ltc.cfgSetDischargeTimeout(LTC68041<ltc_count>::DischargeTimeout::DISCHRG_TIMEOUT_5MIN);
    _ltc.cfgWrite();
}

bool LtcMebWrapper::detect_battery() {
    std::array<float, 12> voltages;
    bool ret;

    init();
    measure_cells();

    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            ret = _ltc.getCellVoltages<12,0>(voltages);
            break;
            case 1:
            ret = _ltc.getCellVoltages<12,1>(voltages);
            break;
            default:
            break;
        }
    } else {
        ret = _ltc.getCellVoltages(voltages);
    }
    
    _bat_type = detect_battery_type(voltages);

    return ret;
}

void LtcMebWrapper::set_battery_type(BatteryType type) {
    _bat_type = type;
}

BatteryType LtcMebWrapper::battery_type() {
    return _bat_type;
}

constexpr float LtcMebWrapper::raw_voltage_to_real_temp(float raw_voltage, float r2, float r0, int beta) {
    //return 32.0513f * raw_voltage - 23.0769f;

    float r = (r2 * (3.0f - raw_voltage)) / raw_voltage;

    float inv_t = 1.0f / 298.15f + (1 / beta) * log(r / r0);
    float t = (1.0f / inv_t) - 273.15f;
    return t;
}

void LtcMebWrapper::set_balance_bits(const std::vector<bool> &balance_bits) {
    _balance_error = false;

    std::bitset<12> bits;

    switch(_bat_type)
    {
        case BatteryType::meb8s:
            if(balance_bits.size() != 8)
                return;
            
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
            break;
        case BatteryType::meb12s:
            if(balance_bits.size() != 12)
                return;

            for (int i = 0; auto bit : balance_bits) {
                bits[i++] = bit;
            }
            break;
        default:
            return;
    }

    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            _ltc.cfgSetDCC<0>(bits);
            break;
            case 1:
            _ltc.cfgSetDCC<1>(bits);
            break;
            default:
            break;
        }
    } else {
        _ltc.cfgSetDCC(bits);
    }

    _ltc.cfgWrite();

    if(_balance_error = !_ltc.cfgRead())
        return;

    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            _balance_error = !(bits == _ltc.cfgGetDCC<0>());
            break;
            case 1:
            _balance_error = !(bits == _ltc.cfgGetDCC<1>());
            break;
            default:
            break;
        }
    } else {
        _balance_error = !(bits == _ltc.cfgGetDCC());
    }

    if (!_balance_error) {
        _balancing = bits.any();
    }
}

std::vector<bool> LtcMebWrapper::get_balance_bits() {
    std::bitset<12> bits;
    std::vector<bool> balance_bits(12);

    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            bits = _ltc.cfgGetDCC<0>();
            break;
            case 1:
            bits = _ltc.cfgGetDCC<1>();
            break;
            default:
            break;
        }
    } else {
        bits = _ltc.cfgGetDCC();
    }

    switch(_bat_type)
    {
        case BatteryType::meb8s:
            balance_bits.resize(8);

            balance_bits[0] = bits[0];
            balance_bits[1] = bits[1];
            balance_bits[2] = bits[2];
            balance_bits[3] = bits[3];

            balance_bits[4] = bits[8]; 
            balance_bits[5] = bits[9]; 
            balance_bits[6] = bits[10];
            balance_bits[7] = bits[11];
            break;
        case BatteryType::meb12s:
            for (int i = 0; i < balance_bits.size(); i++) {
                balance_bits[i] = bits[i];
            }
            break;
        default:
            return std::vector<bool>();
    }

    return balance_bits;
}

bool LtcMebWrapper::is_balancing() {
    return _balancing;
}

void LtcMebWrapper::measure_cells() {
    if (_ltc_index != 0)
        return;

    unsigned long sleep = _ltc.startCellSocConv(LTC68041<ltc_count>::DCP_DISABLED);

    if (sleep)
        delay(sleep);
}

void LtcMebWrapper::measure_temps() {
    if (_ltc_index != 0)
        return;

    unsigned long sleep = _ltc.startAuxConv();

    if (sleep)
        delay(sleep);

    sleep = _ltc.startStatusConv(LTC68041<ltc_count>::CHST_ITMP);

    if (sleep)
        delay(sleep);
}

void LtcMebWrapper::measure_aux() {
    if (_ltc_index != 0)
        return;

    unsigned long sleep;

    if constexpr (ltc_count > 1) {
        sleep = _ltc.startAuxConv(LTC68041<ltc_count>::AuxChannel::CHG_GPIO1);
    } else {
        sleep = _ltc.startAuxConv(LTC68041<ltc_count>::AuxChannel::CHG_GPIO5);
    }

    if (sleep)
        delay(sleep);
}

std::vector<float> LtcMebWrapper::module_temps() {
    std::vector<float> temps;
    DEBUG_PRINTLN("read module temps");

    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            temps.push_back(raw_voltage_to_real_temp(_ltc.getAuxVoltage<0>(LTC68041<ltc_count>::AuxChannel::CHG_GPIO2), 12000.0f, 10000.0f, 3380));
            temps.push_back(raw_voltage_to_real_temp(_ltc.getAuxVoltage<0>(LTC68041<ltc_count>::AuxChannel::CHG_GPIO3), 12000.0f, 10000.0f, 3380));
            break;
            case 1:
            temps.push_back(raw_voltage_to_real_temp(_ltc.getAuxVoltage<1>(LTC68041<ltc_count>::AuxChannel::CHG_GPIO2), 12000.0f, 10000.0f, 3380));
            temps.push_back(raw_voltage_to_real_temp(_ltc.getAuxVoltage<1>(LTC68041<ltc_count>::AuxChannel::CHG_GPIO3), 12000.0f, 10000.0f, 3380));
            break;
            default:
            break;
        }
    } else {
        temps.push_back(raw_voltage_to_real_temp(_ltc.getAuxVoltage(LTC68041<ltc_count>::AuxChannel::CHG_GPIO1), 12000.0f, 10000.0f, 3380));
        temps.push_back(raw_voltage_to_real_temp(_ltc.getAuxVoltage(LTC68041<ltc_count>::AuxChannel::CHG_GPIO2), 12000.0f, 10000.0f, 3380));
    }
    
    return temps;
}

std::vector<float> LtcMebWrapper::pcb_temps() {
    std::vector<float> temps;
    DEBUG_PRINTLN("read pcb temps");

    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            temps.push_back(raw_voltage_to_real_temp(3.0f - _ltc.getAuxVoltage<0>(LTC68041<ltc_count>::AuxChannel::CHG_GPIO4), 10000.0f, 10000.0f, 3380));
            temps.push_back(raw_voltage_to_real_temp(3.0f - _ltc.getAuxVoltage<0>(LTC68041<ltc_count>::AuxChannel::CHG_GPIO5), 10000.0f, 10000.0f, 3380));
            break;
            case 1:
            temps.push_back(raw_voltage_to_real_temp(3.0f - _ltc.getAuxVoltage<1>(LTC68041<ltc_count>::AuxChannel::CHG_GPIO4), 10000.0f, 10000.0f, 3380));
            temps.push_back(raw_voltage_to_real_temp(3.0f - _ltc.getAuxVoltage<1>(LTC68041<ltc_count>::AuxChannel::CHG_GPIO5), 10000.0f, 10000.0f, 3380));
            break;
            default:
            break;
        }
    } else {
        temps.push_back(raw_voltage_to_real_temp(3.0f - _ltc.getAuxVoltage(LTC68041<ltc_count>::AuxChannel::CHG_GPIO3), 10000.0f, 10000.0f, 3380));
        temps.push_back(raw_voltage_to_real_temp(3.0f - _ltc.getAuxVoltage(LTC68041<ltc_count>::AuxChannel::CHG_GPIO4), 10000.0f, 10000.0f, 3380));
    }
    
    return temps;
}

float LtcMebWrapper::module_voltage() {
    DEBUG_PRINTLN("read soc");
    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            return _ltc.getStatusVoltage<0>(LTC68041<ltc_count>::CHST_SOC);
            case 1:
            return _ltc.getStatusVoltage<1>(LTC68041<ltc_count>::CHST_SOC);
            default:
            return NAN;
            break;
        }
    } else {
        return _ltc.getStatusVoltage(LTC68041<ltc_count>::CHST_SOC);
    }
}

float LtcMebWrapper::chip_temp() {
    DEBUG_PRINTLN("read itmp");
    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            return _ltc.getStatusVoltage<0>(LTC68041<ltc_count>::CHST_ITMP);
            case 1:
            return _ltc.getStatusVoltage<1>(LTC68041<ltc_count>::CHST_ITMP);
            default:
            return NAN;
            break;
        }
    } else {
        return _ltc.getStatusVoltage(LTC68041<ltc_count>::CHST_ITMP);
    }
}

std::vector<float> LtcMebWrapper::cell_voltages() {
    DEBUG_PRINTLN("read cells");

    switch(_bat_type)
    {
        case BatteryType::meb12s:
            return get_cells<12>();
        case BatteryType::meb8s:
            return get_cells<8>();
        default:
            return std::vector<float>();
    }
}

float LtcMebWrapper::aux_voltage() {
    if constexpr (ltc_count > 1) {
        switch (_ltc_index) {
            case 0:
            return _ltc.getAuxVoltage<0>(LTC68041<ltc_count>::CHG_GPIO1);
            case 1:
            return _ltc.getAuxVoltage<1>(LTC68041<ltc_count>::CHG_GPIO1);
            default:
            return NAN;
            break;
        }
    } else {
        return _ltc.getAuxVoltage(LTC68041<ltc_count>::CHG_GPIO5);
    }
}

bool LtcMebWrapper::measure_error() {
    return _measure_error;
}

bool LtcMebWrapper::balance_error() {
    return _balance_error;
}
