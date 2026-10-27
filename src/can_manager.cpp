#include "can_manager.h"
#include "mqtt_manager.h"

#include <cstring>

ESP32_TWAI CanManager::can_bus{0, static_cast<gpio_num_t>(Config::can_tx_pin), static_cast<gpio_num_t>(Config::can_rx_pin)};

float CanManager::number_of_cells = static_cast<float>(Config::battery_modules * Config::battery_cells_per_module);

float CanManager::limit_battery_voltage_max;
float CanManager::limit_battery_voltage_min;
float CanManager::limit_discharge_current_max;
float CanManager::limit_charge_current_max;

float CanManager::battery_voltage;
float CanManager::battery_current;
float CanManager::battery_temp;

float CanManager::cell_temp_max;
float CanManager::cell_temp_min;

float CanManager::soc_percent;
float CanManager::soh_percent;
float CanManager::remaining_capacity_ah;
float CanManager::full_capacity_ah;

unsigned long CanManager::last_successful_send = 0;

unsigned long CanManager::last_send_2s;
unsigned long CanManager::last_send_10s;
unsigned long CanManager::last_send_60s;

bool CanManager::init_failed = false;

std::map<String, ValueConfig> CanManager::value_map = {
    {"limits/max_voltage", {&limit_battery_voltage_max, Config::max_cell_voltage* number_of_cells}},
    {"limits/min_voltage", {&limit_battery_voltage_min, Config::min_cell_voltage* number_of_cells}},
    {"limits/max_discharge_current", {&limit_discharge_current_max, Config::max_current}},
    {"limits/max_charge_current", {&limit_charge_current_max, Config::max_current}},
    {"battery/voltage", {&battery_voltage, Config::default_cell_voltage* number_of_cells}},
    {"battery/current", {&battery_current, 0.f}},
    {"battery/temp", {&battery_temp, 12.f}},
    {"battery/max_cell_temp", {&cell_temp_max, 13.f}},
    {"battery/min_cell_temp", {&cell_temp_min, 11.f}},
    {"battery/soc", {&soc_percent, 50.f}},
    {"battery/soh", {&soh_percent, 100.f}},
    {"battery/remaining_capacity_ah", {&remaining_capacity_ah, 80.f}},
    {"battery/full_capacity_ah", {&full_capacity_ah, 160.f}},
};

template <typename T>
void setBytes(uint8_t* data, const size_t start, T value) {
  const auto value_bytes = reinterpret_cast<uint8_t*>(&value);
  for (size_t i = 0; i < sizeof(T); i++) {
    data[start + i] = value_bytes[sizeof(T) - i - 1];  // big-endian
  }
}

template <typename T>
T getValue(const uint8_t* data, const size_t start) {
  T value{};
  const auto value_bytes = reinterpret_cast<uint8_t*>(&value);
  for (size_t i = 0; i < sizeof(T); ++i) {
    value_bytes[sizeof(T) - 1 - i] = data[start + i];  // big-endian
  }
  return value;
}

void CanManager::init() {
  for (auto& [key, value] : value_map) {
    *value.valuePtr = value.defaultValue;
  }
  init_failed = !can_bus.begin(CanBitRate::BR_500k);
  const unsigned long now = millis();
  last_send_2s = now;
  last_send_10s = now + 333UL;
  last_send_60s = now + 667UL;
}

bool CanManager::send(const CanMsg& message) {
  digitalWrite(LED_BUILTIN, LED_ON);
  bool send_successful = can_bus.write(message) > 0;
  digitalWrite(LED_BUILTIN, LED_OFF);
  if (send_successful) {
    last_successful_send = millis();
  }
  return send_successful;
}

void CanManager::loop() {
  if (init_failed) {
    if (millis() >= 5UL * 60UL * 1000UL) {
      MqttManager::log("can init failed - restarting!", false);
      ESP.restart();
    }
    return;
  }
  while (can_bus.available() > 0) {
    readMessage(can_bus.read());
  }
  if (millis() - last_send_2s >= 2UL * 1000UL) {
    last_send_2s = millis();
    sendLimits();
  }
  if (millis() - last_send_10s >= 10UL * 1000UL) {
    last_send_10s = millis();
    sendCellInfo();
    sendBatteryInfo();
    sendStates();
  }
  if (millis() - last_send_60s >= 60UL * 1000UL) {
    last_send_60s = millis();
    sendAlarm();
  }
}

void CanManager::sendLimits() {
  float limit_discharge = limit_discharge_current_max;
  float limit_charge = limit_charge_current_max;
  if (millis() - MqttManager::last_master_heartbeat_time >= heartbeat_timeout_limits_ms) {
    limit_discharge = 0;
    limit_charge = 0;
    MqttManager::log("Master Heartbeat missed!");
  }
  CanMsg message;
  message.id = 0x110;
  message.data_length = 8;
  setBytes(message.data, 0, static_cast<uint16_t>(limit_battery_voltage_max * 10.f));  // 230V max
  setBytes(message.data, 2, static_cast<uint16_t>(limit_battery_voltage_min * 10.f));  // 170V min
  setBytes(message.data, 4, static_cast<uint16_t>(limit_discharge * 10.f));            // 25,6A max discharge
  setBytes(message.data, 6, static_cast<uint16_t>(limit_charge * 10.f));               // 25,6A max charge
  if (send(message)) {
    MqttManager::publish("limits/max_voltage", limit_battery_voltage_max);
    MqttManager::publish("limits/min_voltage", limit_battery_voltage_min);
    MqttManager::publish("limits/max_discharge_current", limit_discharge);
    MqttManager::publish("limits/max_charge_current", limit_charge);
  }
}

void CanManager::sendBatteryInfo() {
  CanMsg message;
  message.id = 0x1d0;
  message.data_length = 8;
  setBytes(message.data, 0, static_cast<int16_t>(battery_voltage * 10.f));  // 215V battery voltage
  setBytes(message.data, 2, static_cast<int16_t>(battery_current * 10.f));  // 4,3A battery current
  setBytes(message.data, 4, static_cast<int16_t>(battery_temp * 10.f));     // 22°C battery temp
  setBytes(message.data, 6, static_cast<int16_t>(776));                     // some sort of status bytes?
  if (send(message)) {
    MqttManager::publish("battery/voltage", battery_voltage);
    MqttManager::publish("battery/current", battery_current);
    MqttManager::publish("battery/temp", battery_temp);
  }
}

void CanManager::sendCellInfo() {
  CanMsg message;
  message.id = 0x210;
  message.data_length = 8;
  setBytes(message.data, 0, static_cast<uint16_t>(cell_temp_max * 10.f));  // 23°C max cell temp
  setBytes(message.data, 2, static_cast<uint16_t>(cell_temp_min * 10.f));  // 22°C min cell temp
  if (send(message)) { // sungrow not checking data?
    MqttManager::publish("battery/max_cell_temp", cell_temp_max);
    MqttManager::publish("battery/min_cell_temp", cell_temp_min);
  }
}

void CanManager::sendStates() {
  remaining_capacity_ah = soc_percent / 100 * full_capacity_ah;  // calculate remaining_capacity_ah by soc
  CanMsg message;
  message.id = 0x150;
  message.data_length = 8;
  setBytes(message.data, 0, static_cast<uint16_t>(soc_percent * 100.f));           // 28,7% soc % Vrfd
  setBytes(message.data, 2, static_cast<uint16_t>(soh_percent * 100.f));           // 100% soh % Vrfd
  setBytes(message.data, 4, static_cast<uint16_t>(remaining_capacity_ah * 10.f));  // remaining capacity 1/10Ah (ignored by sungrow?)
  setBytes(message.data, 6, static_cast<uint16_t>(full_capacity_ah * 10.f));       // fully charged capacity 1/10Ah (ignored by sungrow?)
  if (send(message)) {
    MqttManager::publish("battery/soc", soc_percent);
    MqttManager::publish("battery/soh", soh_percent);
    MqttManager::publish("battery/remaining_capacity_ah", remaining_capacity_ah);
    MqttManager::publish("battery/full_capacity_ah", full_capacity_ah);
  }
}

void CanManager::sendAlarm() {
  CanMsg message;
  message.id = 0x190;
  message.data_length = 8;
  send(message);
}

String canMessageToString(const CanMsg& message) {
  const bool extended = message.isExtendedId();
  const uint8_t data_length = message.data_length;

  char idbuf[9];
  const uint32_t identifier = extended ? message.getExtendedId() : message.getStandardId();
  snprintf(idbuf, sizeof(idbuf), extended ? "%08X" : "%03X", static_cast<unsigned int>(identifier));

  String s;
  s.reserve(9 + 1 + data_length * 2);
  s += idbuf;
  s += '#';

  for (uint8_t i = 0; i < data_length; i++) {
    char b[3];
    snprintf(b, sizeof(b), "%02X", message.data[i]);
    s += b;
  }
  return s;
}

void CanManager::readMessage(const CanMsg& message) {
  const bool extended = message.isExtendedId();
  const uint32_t rxId = extended ? message.getExtendedId() : message.getStandardId();
  const uint8_t len = message.data_length;
  uint8_t rxBuf[9] = {};
  std::memcpy(rxBuf, message.data, len);

  Serial.print("recv: ");
  if (extended) {
    Serial.printf("Extended ID: 0x%.8lX  DLC: %1d  Data:", rxId, len);
  } else {
    Serial.printf("Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  }
  for (byte i = 0; i < len; i++) {
    Serial.printf(" 0x%.2X", rxBuf[i]);
  }
  Serial.println();

  if (rxId == 0x91) {
    const auto wr_battery_voltage = getValue<uint16_t>(rxBuf, 0);
    const auto wr_battery_current = getValue<uint16_t>(rxBuf, 2); // int16_t ?
    const auto wr_temperature = getValue<uint16_t>(rxBuf, 4);
    MqttManager::publish("inverter/battery_voltage", static_cast<float>(wr_battery_voltage) * 0.1f);
    MqttManager::publish("inverter/battery_current", static_cast<float>(wr_battery_current) * 0.1f);
    MqttManager::publish("inverter/temperature", static_cast<float>(wr_temperature) * 0.1f);
  } else if (rxId == 0xd1) {
    const auto wr_soc = getValue<uint16_t>(rxBuf, 0);
    MqttManager::publish("inverter/soc", static_cast<float>(wr_soc) * 0.1f);
  } else if (rxId == 0x111) {
    const auto wr_timestamp = getValue<uint32_t>(rxBuf, 0);
    MqttManager::publish("inverter/timestamp", wr_timestamp);
  } else if (rxId == 0x151 && rxBuf[0] == 0x0) {
    rxBuf[len] = '\0';
    if (const auto inverter_name = String(reinterpret_cast<char*>(rxBuf + 1)); inverter_name.length() > 0) {
      MqttManager::publish("inverter/type", String(reinterpret_cast<char*>(rxBuf + 1)), true);
    }
  } else if (rxId == 0x151 && rxBuf[0] == 0x1) {
    MqttManager::log("sending initMessages!");
    for (const auto& [id, data] : initMessages) {
      const CanMsg message(id, 8, data);
      for (int attempts = 0; attempts < 3 && !send(message); attempts++) {
        delayMicroseconds(10);
      }
    }
  } else {
    MqttManager::log(canMessageToString(message));
  }
}
