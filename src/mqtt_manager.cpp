#include "mqtt_manager.h"

#include <map>
#include <set>
#include <charconv>

#include "can_manager.h"
#include "wifi.hpp"
#include "Wifi.h"

PsychicMqttClient MqttManager::client;

String MqttManager::module_topic = "master/can/";
String MqttManager::will_topic;
String MqttManager::client_id;
unsigned long MqttManager::last_blink_time = 0;
unsigned long MqttManager::last_master_heartbeat_time = 0;

void MqttManager::init(const char *const server, const char *const user,
                       const char *const password)
{
  client_id = String("espcan-") + WifiManager::mac_string();

  if (module_topic.length() <= 0) {
    module_topic = client_id + "/";
  }

  client.setServer(server);
  client.setCredentials(user, password);
  will_topic = module_topic + "available";
  client.setClientId(client_id.c_str());
  client.setWill(will_topic.c_str(), 0, true, "offline");
  client.setKeepAlive(60);
  client.onConnect(onConnect);
  client.onMessage(onMessage);
  client.connect();
  last_master_heartbeat_time = millis();
}

void MqttManager::loop() {
  unsigned long last_heartbeat = last_master_heartbeat_time;
  unsigned long delta = millis() - last_heartbeat;
  if (delta >= heartbeat_timeout_reboot_ms) {
    log(String("DEBUG check: delta=") + String(delta) + ", limit=" + String(heartbeat_timeout_reboot_ms), false);
    log(String(millis()) + " : " + String(last_heartbeat), false);
    log("master heartbeat timeout - restarting!", false);
    ESP.restart();
  }
}

void MqttManager::onMessage(char* topic, char* payload, int retain, int qos, bool dup) {
  String sTopic(topic);
  if (sTopic.startsWith(module_topic)) {
    sTopic = sTopic.substring(module_topic.length());
  }
  if (sTopic.equals("blink")) {
    last_blink_time = millis();
    return;
  }
  if (sTopic.equals(mqtt_master_heartbeat_topic)) {
    last_master_heartbeat_time = millis();
    return;
  }
  if (sTopic.equals("restart")) {
    log("restart requested - restarting!", false);
    ESP.restart();
  }
  const bool isSet = sTopic.endsWith("/set");
  const bool isReset = sTopic.endsWith("/reset");
  float value;
  if (isSet && (std::from_chars(payload, payload + strlen(payload), value).ec != std::errc())) {
    log(String("failed to parse ") + payload + " of topic " + sTopic + ".");
    return;
  }
  if (!isSet && !isReset) return;
  if (isSet) {
    sTopic = sTopic.substring(0, sTopic.length() - 4);
  } else {
    sTopic = sTopic.substring(0, sTopic.length() - 6);
  }
  const auto it = CanManager::value_map.find(sTopic);
  if (it == CanManager::value_map.end()) {
    return;
  }
  if (isSet) {
    *it->second.valuePtr = value;
  } else {
    *it->second.valuePtr = it->second.defaultValue;
  }
}

void MqttManager::log(const String& line, const bool async, const int priority) {
  publish("log", line, false, async, priority);
}

void MqttManager::publish(const String& topic, const float value, const bool retain, const bool async,
                          const int priority) {
  publish(topic, String(value), retain, async, priority);
}

void MqttManager::publish(const String& topic, const uint32_t value, const bool retain, const bool async,
                          const int priority) {
  publish(topic, String(value), retain, async, priority);
}

void MqttManager::publish(const String& topic, const String& payload, const bool retain, const bool async,
                          const int priority) {
    client.publish((module_topic + topic).c_str(), 0, retain, payload.c_str(), 0, async);
}

void MqttManager::subscribe(const String& topic) { client.subscribe((module_topic + topic).c_str(), 0); }

void MqttManager::publishInfos() {
  // publish("version", VERSION, true, false);
  // publish("build_timestamp", BUILD_TIMESTAMP, true, false);
  publish("wifi", WiFi.SSID(), true, false, 20);
  publish("ip", WiFi.localIP().toString(), true, false, 40);
  publish("esp_sdk", ESP.getSdkVersion(), true, false, 20);
  publish("cpu",
          String(ESP.getChipModel()) + " rev " + ESP.getChipRevision() + " " + ESP.getChipCores() + "x" +
              ESP.getCpuFreqMHz() + "MHz",
          true, false, 20);
  publish("flash", String(ESP.getFlashChipSize() / 1024 / 1024) + " MiB, Mode: " + ESP.getFlashChipMode(), true, false,
          20);
  publish("heap", String(ESP.getHeapSize() / 1024) + " KiB", true, false, 20);
  publish("psram", String(ESP.getPsramSize() / 1024) + " KiB", true, false, 20);
  // publish("build_time", unixToTime(CURRENT_TIME), true, false);
}

void MqttManager::onConnect(bool session_present) {
  Serial.println("connected");
  publish("available", "online", true, false, 100);
  publish("hostname", client_id, true, false, 20);
  publish("module_topic", module_topic, true, false, 20);
  publishInfos();
  client.subscribe(mqtt_master_heartbeat_topic, 0);
  subscribe("+/+/set");
  subscribe("+/+/reset");
  subscribe("restart");
  // subscribe("debug");
  subscribe("blink");
}
