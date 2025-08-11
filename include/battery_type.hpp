#pragma once

#include <WString.h>

#include <array>

enum class BatteryConfig { meb12s = 0, meb8s, mebAuto };
enum class BatteryType { meb12s = 0, meb8s };

BatteryType detect_battery_type(const std::array<float, 12>& voltages);
String as_string(BatteryConfig battery_config);
String as_string(BatteryType battery_type);