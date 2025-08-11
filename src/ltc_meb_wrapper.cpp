#include "ltc_meb_wrapper.hpp"

#include <array>

LtcMebWrapper::LtcMebWrapper() : _ltc(18), _balance_error{false}, _measure_error{false} // CSLTC
{
}

void LtcMebWrapper::init() {
    _ltc.initSPI(2, 7, 6); // MOSI, MISO, SCLK

    if (_ltc.checkSPI()) {
        // digitalWrite(D1, HIGH); // LED1
    } else {
        // digitalWrite(D1, LOW); // LED1
    }
}

bool LtcMebWrapper::detect_battery() {
    std::array<float, 12> voltages;
    bool ret;

    _ltc.initSPI(2, 7, 6); // MOSI, MISO, SCLK
    measure_cells();
    ret = _ltc.getCellVoltages(voltages);
    _bat_type = detect_battery_type(voltages);
    _ltc.destroySPI();

    return ret;
}

void LtcMebWrapper::set_battery_type(BatteryType type) {
    _bat_type = type;
}

BatteryType LtcMebWrapper::battery_type() {
    return _bat_type;
}

float LtcMebWrapper::raw_voltage_to_real_module_temp(float raw_voltage) {
    return 32.0513f * raw_voltage - 23.0769f;
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

    if (!_ltc.cfgRead()) {
        _balance_error = true;
        return;
    }

    _ltc.cfgSetRefOn(true);
    _ltc.cfgSetVUV(3.1);
    _ltc.cfgSetVOV(4.2);

    if (bits.any()) {
        // digitalWrite(D2, HIGH); // LED2
    } else {
        // digitalWrite(D2, LOW); // LED2
    }

    _ltc.cfgSetDCC(bits);
    _ltc.cfgWrite();
    _ltc.cfgRead();
    _balance_error = !(bits == _ltc.cfgGetDCC());
}

std::vector<bool> &&LtcMebWrapper::get_balance_bits() {
    std::bitset<12> bits =_ltc.cfgGetDCC();
    std::vector<bool> balance_bits(12);

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
                balance_bits[i] = bits[i++];
            }
            break;
        default:
            return std::vector<bool>();
    }

    return balance_bits;
}

void LtcMebWrapper::measure_cells() {
    _ltc.waitForConversion();
    _ltc.startCellConv(LTC68041::DCP_DISABLED);
}

void LtcMebWrapper::measure_aux() {
    _ltc.waitForConversion();
    _ltc.startAuxConv();
    _ltc.waitForConversion();
    _ltc.startStatusConv();
}

float LtcMebWrapper::module_temp_1() {
    return raw_voltage_to_real_module_temp(_ltc.getAuxVoltage(LTC68041::AuxChannel::CHG_GPIO1));
}

float LtcMebWrapper::module_temp_2() {
    return raw_voltage_to_real_module_temp(_ltc.getAuxVoltage(LTC68041::AuxChannel::CHG_GPIO2));
}

float LtcMebWrapper::module_voltage() {
    return _ltc.getStatusVoltage(LTC68041::CHST_SOC);
}

float LtcMebWrapper::chip_temp() {
    return _ltc.getStatusVoltage(LTC68041::CHST_ITMP);
}

std::vector<float> LtcMebWrapper::cell_voltages() {
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

bool LtcMebWrapper::measure_error() {
    return _measure_error;
}

bool LtcMebWrapper::balance_error() {
    return _balance_error;
}
