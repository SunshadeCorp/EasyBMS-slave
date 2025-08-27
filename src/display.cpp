#include "display.hpp"

#include <SPI.h>

#include <array>

Display::Display() : _tft(TFT_eSPI()) {
    for (auto& row : _old_chars) {
        for (auto& c : row) {
            c = ' ';
        }
    }
    clear();
}

void Display::init() {
    _tft.init();
    _tft.fillScreen(_background_color);
    _tft.setRotation(1);
    _tft.setFreeFont(&Roboto_Mono_Light_13);
}

String Display::format(float value, uint8_t decplaces, float min, float max, String unit) {
    if (value < max && value >= min) {
        return String(value, decplaces) + unit;
    } else {
        return "invld";
    }
}

String Display::format(int value, int min, int max, String unit) {
    if (value < max && value >= min) {
        return String(value) + unit;
    } else {
        return "invld";
    }
}

String Display::format_temp(float value) {
    return format(value, 1, -99.9, 99.9, "C");
}

String Display::format_cell_voltage(float value) {
    return format(value, 3, 0, 9.999);
}

void Display::clear() {
    for (auto& row : _screen_chars) {
        for (auto& c : row) {
            c = ' ';
        }
    }
}

void Display::print(uint8_t column, uint8_t row, String text) {
    for (size_t x = column; x < _screen_chars[row].size(); x++) {
        _screen_chars[row][x] = text[x - column];
    }
}

void Display::flip() {
    for (size_t row = 0; row < _screen_chars.size(); row++) {
        for (size_t column = 0; column < _screen_chars[row].size(); column++) {
            auto c_new = _screen_chars[row][column];
            auto c_old = _old_chars[row][column];
            if (c_new != c_old) {
                // Paint over
                _tft.setTextColor(_background_color);
                _tft.setCursor(_char_width * column, (row + 1) * _char_height);
                _tft.print(_old_chars[row][column]);

                // Paint new
                _tft.setTextColor(_text_color);
                _tft.setCursor(_char_width * column, (row + 1) * _char_height);
                _tft.print(_screen_chars[row][column]);
            }
        }
    }

    _old_chars = _screen_chars;
    clear();
}

void Display::update(std::shared_ptr<BatteryMonitor> m) {
    // Print Cell Voltages
    auto& cell_voltages = m->cell_voltages();
    for (size_t i = 0; i < cell_voltages.size(); i++) {
        if (m->measure_error()) {
            print(0, i, "-----");
        } else {
            String cell_voltage = format_cell_voltage(cell_voltages[i]);
            print(0, i, cell_voltage);
        }
    }

    // Print Balance Bits
    auto& balance_bits = m->balance_bits();
    for (size_t i = 0; i < balance_bits.size(); i++) {
        if (balance_bits[i]) {
            print(5, i, "-");
        }
    }

    // Print Stats
    String cell_diff = format_cell_voltage(m->cell_diff());
    String soc = format(m->soc(), 1, -99.9, 999.9, "%");
    String module_voltage = format(m->module_voltage(), 1, 0, 99.9, "V");
    String min_cell_voltage = format_cell_voltage(m->min_voltage());
    String avg_cell_voltage = format_cell_voltage(m->avg_voltage());
    String max_cell_voltage = format_cell_voltage(m->max_voltage());
    String module_temp_1 = format_temp(m->module_temp_1());
    String module_temp_2 = format_temp(m->module_temp_2());
    String chip_temp = format_temp(m->chip_temp());
    String error_string = m->measure_error() ? "ERROR" : "";

    if (m->measure_error()) {
        min_cell_voltage = "-----";
        max_cell_voltage = "-----";
        avg_cell_voltage = "-----";
        soc = "-----";
        cell_diff = "-----";
        module_voltage = "-----";
    }

    String cell_diff_trend;
    if (m->cell_diff_trend().has_value()) {
        int cell_diff_mv = static_cast<int>(m->cell_diff_trend().value() * 1000);
        cell_diff_trend = format(cell_diff_mv, 0, -99, 99, "mVh");
    } else {
        cell_diff_trend = "-----";
    }

    print(7, 0, "Dif:" + cell_diff);
    // print(7, 1, "Tre:" + cell_diff_trend);
    print(7, 1, "SOC:" + soc);
    print(7, 2, "Mod:" + module_voltage);
    print(7, 3, "Min:" + min_cell_voltage);
    print(7, 4, "Avg:" + avg_cell_voltage);
    print(7, 5, "Max:" + max_cell_voltage);
    print(7, 6, "t1: " + module_temp_1);
    print(7, 7, "t2: " + module_temp_2);
    print(7, 8, "ti: " + chip_temp);
    print(7, 11, error_string);

    flip();
}