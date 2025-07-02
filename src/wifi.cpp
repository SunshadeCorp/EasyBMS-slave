#include "wifi.hpp"

#include <Arduino.h>
#include <HTTPUpdate.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <lwip/dns.h>

#include "debug.hpp"

void connect_wifi(String hostname, String ssid, String password) {
    Serial.println();
    Serial.println("connecting to ");
    Serial.println(ssid);
    WiFi.persistent(false);
    WiFi.softAPdisconnect(true);
    WiFiClass::mode(WIFI_STA);
    WiFiClass::hostname(hostname);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("DNS1: ");
    Serial.println(IPAddress(dns_getserver(0)));
    Serial.println("DNS2: ");
    Serial.println(IPAddress(dns_getserver(1)));

    randomSeed(micros());
#if SSL_ENABLED
    espClient.setTrustAnchors(&mqtt_cert_store);
#endif
}

String mac_string() {
    uint8_t mac[6];
    esp_base_mac_addr_get(mac);
    char mac_string[6 * 2 + 1] = {};
    snprintf(mac_string, sizeof(mac_string), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return {mac_string};
}

String perform_ota_update(String url, const char* cert) {
    NetworkClientSecure client_secure;
    client_secure.setCACert(cert);
    client_secure.setTimeout(12000);
    httpUpdate.setLedPin(LED_BUILTIN, HIGH);
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
    return result_string;
}