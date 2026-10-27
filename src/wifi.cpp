#include "wifi.hpp"

#include <Arduino.h>
#include <HTTPUpdate.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <lwip/dns.h>

#include "debug.hpp"

#ifndef LED_STATUS
#define LED_STATUS = LED_BUILTIN
#endif

void WifiManager::connect(String hostname, String ssid, String password) {
    Serial.println();
    Serial.printf("Connecting to %s.", ssid);
    WiFi.persistent(false);
    WiFi.softAPdisconnect(true);
    WiFi.setAutoReconnect(true);
    WiFiClass::mode(WIFI_STA);
    WiFiClass::hostname(hostname);
    WiFi.begin(ssid, password);
    #ifdef LOLIN_C3_MINI_V1
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    #endif
    const unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start >= wifi_timeout_ms) {
            ESP.restart();
        }
        delay(500);
        Serial.print(".");
    }
    WiFi.setSleep(true);
    Serial.println("\nWiFi connected!");
    const String localIP = WiFi.localIP().toString();
    Serial.printf("IP Address: %s\n", localIP.c_str());

    for (int i = 0; i < 2; i++) {
        const String dns = WiFi.dnsIP(i).toString();
        Serial.printf("DNS%d: %s\n", i + 1, dns.c_str());
    }
}

String WifiManager::mac_string() {
    uint8_t mac[6];
    esp_base_mac_addr_get(mac);
    char mac_string[6 * 2 + 1] = {};
    snprintf(mac_string, sizeof(mac_string), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return {mac_string};
}

String WifiManager::perform_ota_update(String url, const char* cert) {
    NetworkClientSecure client_secure;
    client_secure.setCACert(cert);
    client_secure.setTimeout(12000);
    WiFi.setSleep(false);
    httpUpdate.setLedPin(LED_STATUS, HIGH);
    auto result = httpUpdate.update(client_secure, String("https://") + url);
    String result_string;
    switch (result) {
        case HTTP_UPDATE_FAILED:
            result_string = String("HTTP_UPDATE_FAILED Error (");
            result_string += httpUpdate.getLastError();
            result_string += "): ";
            result_string += httpUpdate.getLastErrorString();
            result_string += "\n";
            break;
        case HTTP_UPDATE_NO_UPDATES:
            result_string = "HTTP_UPDATE_NO_UPDATES";
            break;
        case HTTP_UPDATE_OK:
            result_string = "HTTP_UPDATE_OK";
            break;
    }
    DEBUG_PRINTLN(result_string);
    WiFi.setSleep(true);
    return result_string;
}

void WifiManager::syncTime() {
    Serial.print("Setting time using SNTP.");
    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "0.de.pool.ntp.org", "1.de.pool.ntp.org", "2.de.pool.ntp.org");
    time_t now = time(nullptr);
    const unsigned long start = millis();
    while (now < min_time_s && millis() - start < sntp_timeout_ms) {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    if (now < min_time_s) {
        Serial.println("\nSNTP-Sync timeout!");
        return;
    }
    Serial.println("\nTime synchronized!");
    tm timeInfo{};
    getLocalTime(&timeInfo);
    char current_time_buffer[32];
    strftime(current_time_buffer, sizeof(current_time_buffer), "%c", &timeInfo);
    Serial.printf("Current time: %s", current_time_buffer);
}