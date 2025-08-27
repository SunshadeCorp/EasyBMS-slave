#pragma once

#include <TFT_eSPI.h>

#include <array>

#include "battery_monitor.hpp"

/*
#define TFT_CS   19
#define TFT_RST  4  // TFT RST = Arduino RST
#define TFT_DC   5
#define TFT_SCLK 6
#define TFT_MOSI 2
*/

class Display {
   public:
    Display();

    // Initialize the display, sets rotation and font size etc.
    void init();

    // Main function to print all BMS data
    void update(std::shared_ptr<BatteryMonitor> monitor);

   private:
    const uint8_t _char_height = 10;
    const uint8_t _char_width = 8;
    const uint16_t _text_color = 0xFFFF;
    const uint16_t _background_color = 0x0000;

    std::array<std::array<char, 22>, 12> _old_chars;
    std::array<std::array<char, 22>, 12> _screen_chars;

    // Configured in User_Setup.h
    TFT_eSPI _tft;

    String format(float value, uint8_t decplaces, float min, float max, String unit = "");
    String format(int value, int min, int max, String unit = "");
    String format_temp(float value);
    String format_cell_voltage(float value);
    void clear();
    void print(uint8_t column, uint8_t row, String text);
    void flip();
};