#pragma once

#include <Arduino.h>
#include <PsychicMqttClient.h>

class MqttManager {
 public:
  static void init(const char *const server, const char *const user,
                   const char *const password);
  static void loop();
  static void log(const String& line, bool async = true, int priority = 10);
  static void publish(const String& topic, float value, bool retain = false, bool async = true, int priority = 0);
  static void publish(const String& topic, uint32_t value, bool retain = false, bool async = true, int priority = 0);
  static void publish(const String& topic, const String& payload, bool retain = false, bool async = true,
                      int priority = 0);
  static void subscribe(const String& topic);
  static void publishInfos();
  static unsigned long last_master_heartbeat_time;

 private:
  static constexpr const char* mqtt_master_heartbeat_topic = "master/uptime";
  static constexpr unsigned int max_mqtt_send_queue = 100;
  static constexpr unsigned long heartbeat_timeout_limits_ms = 2UL * 60UL * 1000UL;
  static constexpr unsigned long heartbeat_timeout_reboot_ms = 10UL * 60UL * 1000UL;
  static PsychicMqttClient client;
  static String module_topic, will_topic, client_id;
  static unsigned long last_blink_time;
  static void onConnect(bool session_present);
  static void onMessage(char* topic, char* payload, int retain, int qos, bool dup);
  static void otaUpdate(const String& path);
};
