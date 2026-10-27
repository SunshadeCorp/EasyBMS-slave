#pragma once

#include <Arduino.h>
#include <ESP32_TWAI.h>

#include "config.h"

#include <map>

struct ValueConfig {
  float* valuePtr;
  float defaultValue;
};

class CanManager {
 public:
  static constexpr uint32_t CAN_EXTENDED = 0x80000000;
  static constexpr uint32_t CAN_REMOTE_REQUEST = 0x40000000;

  static void init();
  static bool send(const CanMsg& message);
  static void loop();
  static void readMessage(const CanMsg& message);

  static std::map<String, ValueConfig> value_map;

  static float number_of_cells;

  static float limit_battery_voltage_max;
  static float limit_battery_voltage_min;
  static float limit_discharge_current_max;
  static float limit_charge_current_max;

  static float battery_voltage;
  static float battery_current;
  static float battery_temp;

  static float cell_temp_max;
  static float cell_temp_min;

  static float soc_percent;
  static float soh_percent;
  static float remaining_capacity_ah;
  static float full_capacity_ah;

 private:
  struct Message {
    unsigned long id;
    byte data[8];
  };

  static ESP32_TWAI can_bus;
  static constexpr unsigned long heartbeat_timeout_limits_ms = 2UL * 60UL * 1000UL;
  static constexpr uint8_t fw_major_version = 0x03;
  static constexpr uint8_t fw_minor_version = 0x16;
  static constexpr Message initMessages[] = {
      {0x250,
      {fw_major_version, fw_minor_version, 0x00, 0x66, static_cast<uint8_t>((Config::battery_wh_max / 100) >> 8),
        static_cast<uint8_t>(Config::battery_wh_max / 100), 0x02, 0x09}},
      {0x290, {0x06, 0x37, 0x10, 0xD9, 0x00, 0x00, 0x00, 0x00}},
      {0x2D0, {0x00, 'B', 'Y', 'D', 0x00, 0x00, 0x00, 0x00}},
      {0x3D0, {0x00, 'B', 'a', 't', 't', 'e', 'r', 'y'}},
      {0x3D0, {0x01, '-', 'B', 'o', 'x', ' ', 'P', 'r'}},
      {0x3D0, {0x02, 'e', 'm', 'i', 'u', 'm', ' ', 'H'}},
      {0x3D0, {0x03, 'V', 'S', 0x00, 0x00, 0x00, 0x00, 0x00}},
  };

  static bool init_failed;
  static unsigned long last_successful_send;
  static unsigned long last_send_2s;
  static unsigned long last_send_10s;
  static unsigned long last_send_60s;
  static void sendLimits();
  static void sendStates();
  static void sendBatteryInfo();
  static void sendCellInfo();
  static void sendAlarm();
};
