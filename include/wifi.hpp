#pragma once

#include <WString.h>
#include <IPAddress.h>

String perform_ota_update(String url, const char* cert);
class WifiManager {
 public:
  static void connect(String hostname, String ssid, String password);
  static void syncTime();
  static void activate_modem_sleep();
  static void deactivate_modem_sleep();
  static String mac_string();
  static String perform_ota_update(String url, const char* cert);

 private:
   static constexpr unsigned long sntp_timeout_ms = 10UL * 1000UL;
   static constexpr unsigned long wifi_timeout_ms = 30UL * 1000UL;
   static constexpr unsigned int min_time_s = 24U * 60U * 60U;
  static IPAddress getDnsServer(int index);
};