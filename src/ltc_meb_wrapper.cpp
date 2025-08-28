#include "ltc_meb_wrapper.hpp"

#include <LTC68041.cpp>  // used for template functions

#include "debug.hpp"

LtcMebWrapper::LtcMebWrapper() {
    _debug_mode = DEBUG;
    _measure_error = false;
    _balance_error = false;
}

void LtcMebWrapper::init() {
    _ltc.initSPI(D7, D6, D5);
}

float LtcMebWrapper::raw_voltage_to_real_module_temp(float raw_voltage) {
    return 32.0513f * raw_voltage - 23.0769f;
}

void LtcMebWrapper::set_balance_bits(const std::bitset<12> &balance_bits) {
    _balance_error = false;

    if (_ltc.checkSPI(_debug_mode)) {
        digitalWrite(D1, HIGH);
    } else {
        digitalWrite(D1, LOW);
    }

    _ltc.cfgSetRefOn(true);
    _ltc.cfgSetVUV(3.1);
    _ltc.cfgSetVOV(4.2);

    if (balance_bits.any()) {
        digitalWrite(D2, HIGH);
    } else {
        digitalWrite(D2, LOW);
    }

    _ltc.cfgSetDCC(balance_bits);
    _ltc.cfgWrite();

    // Start different Analog-Digital-Conversions in the Chip
    _ltc.startCellConv(LTC68041::DCP_DISABLED);
    delay(5);  // Wait until conversion is finished
    _ltc.startAuxConv();
    delay(5);  // Wait until conversion is finished
    _ltc.startStatusConv();
    delay(5);  // Wait until conversion is finished
    if (!_ltc.cfgRead()) {
        _balance_error = true;
    }

    // Print the clear text values cellVoltage, gpioVoltage, Undervoltage Bits, Overvoltage Bits
    if (_debug_mode) {
        _ltc.readCfgDbg();
        _ltc.readStatusDbg();
        _ltc.readAuxDbg();
        _ltc.readCellsDbg();
    }
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
    return _ltc.getStatusVoltage(_ltc.StatusGroup::CHST_ITMP);
}

std::array<float, 12> LtcMebWrapper::cell_voltages() {
    std::array<float, 12> voltages;
    bool success = _ltc.getCellVoltages<12>(voltages);
    if (success) {
        _measure_error = false;
        voltages[0] += 0.004f;
    } else {
        _measure_error = true;
    }

    return voltages;
}

bool LtcMebWrapper::measure_error() {
    return _measure_error;
}

bool LtcMebWrapper::balance_error() {
    return _balance_error;
}
