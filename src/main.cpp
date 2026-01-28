#include "Arduino.h"
#include "balancer_interface.hpp"
#include "battery_monitor.hpp"
#include "config.h"
#include "debug.hpp"
#include "display.hpp"
#include "ltc_meb_wrapper.hpp"
#include "mock_mqtt_client.hpp"
#include "mqtt_adapter.hpp"
#include "mqtt_client.hpp"
#include "simulated_battery.hpp"
#include "single_mode_balancer.hpp"
#include "timed_history.hpp"
#include "wifi.hpp"

#include <ESP32_TWAI.h>
#include <VEBus.h>

VEBus vebus;

// -----------------------------------------------------------------------
// Device state name helper
// -----------------------------------------------------------------------
static const char *deviceStateName(uint8_t state)
{
    switch (state) {
    case VEBUS_STATE_DOWN:         return "Down";
    case VEBUS_STATE_STARTUP:      return "Startup";
    case VEBUS_STATE_OFF:          return "Off";
    case VEBUS_STATE_SLAVE:        return "Slave";
    case VEBUS_STATE_INVERT_FULL:  return "Invert Full";
    case VEBUS_STATE_INVERT_HALF:  return "Invert Half";
    case VEBUS_STATE_INVERT_AES:   return "Invert AES";
    case VEBUS_STATE_POWER_ASSIST: return "Power Assist";
    case VEBUS_STATE_BYPASS:       return "Bypass";
    case VEBUS_STATE_CHARGE:       return "Charge";
    default:                       return "Unknown";
    }
}

static const char *chargeSubStateName(uint8_t sub)
{
    switch (sub) {
    case VEBUS_CHARGE_INIT:                return "Init";
    case VEBUS_CHARGE_BULK:                return "Bulk";
    case VEBUS_CHARGE_ABSORPTION:          return "Absorption";
    case VEBUS_CHARGE_FLOAT:               return "Float";
    case VEBUS_CHARGE_STORAGE:             return "Storage";
    case VEBUS_CHARGE_REPEATED_ABSORPTION: return "Repeat Abs";
    case VEBUS_CHARGE_FORCED_ABSORPTION:   return "Forced Abs";
    case VEBUS_CHARGE_EQUALISE:            return "Equalise";
    case VEBUS_CHARGE_BULK_STOPPED:        return "Bulk Stopped";
    default:                               return "Unknown";
    }
}

std::array<std::shared_ptr<BMS>, ltc_count> bmsArr;
std::array<std::shared_ptr<MqttAdapter>, ltc_count> mqtt_adapterArr;

#define RAM_OFFSET_MS      500   // request data 500 ms before print
#define EXT_RAM1_OFFSET_MS   300   // 1st extended RAM read
#define EXT_RAM2_OFFSET_MS   200   // 2nd extended RAM read
#define STATE_OFFSET_MS     400   // device state request
#define SETTINGS_OFFSET_MS     100   // device state request
#define PRINT_INTERVAL_MS 1000   // print status every 1 s

// Extended RAM batch 1: voltages, currents, power
volatile int16_t g_mainsVoltage    = 0;
volatile int16_t g_mainsCurrent    = 0;
volatile int16_t g_inverterVoltage = 0;
volatile int16_t g_inverterCurrent = 0;
volatile int16_t g_outputPower     = 0;
volatile int16_t g_mainsPower      = 0;

// Extended RAM batch 2: battery, frequency, SoC
volatile int16_t g_batteryCurrent  = 0;
volatile int16_t g_chargeState     = 0;  // SoC raw value
volatile int16_t g_mainsFreq       = 0;  // period raw (Hz = 10/value)
volatile int16_t g_inverterFreq    = 0;  // period raw (Hz = 10/value)

// Track which RAM batch the response belongs to
volatile uint8_t g_lastRamBatch = 0;

// #define MOCK_BATTERY
// #define MOCK_MQTT

[[maybe_unused]] void setup() {
    DEBUG_BEGIN(74880);
    DEBUG_PRINTLN("init start");

    vebus.begin(20, 21, -1);

    if constexpr (use_can) {
        CAN.begin(CanBitRate::BR_500k);
    }

    DEBUG_PRINTLN("Listening for RS485 data ...");
    delay(3000);
    return;

    auto hostname = String("easybms-") + mac_string();

#ifdef MOCK_MQTT
    auto mqtt = std::make_shared<MockMqttClient>();
    mqtt->is_connected = false;
    mqtt->connect_result = true;
#else
    auto mqtt = std::make_shared<MqttClient>(mqtt_server, mqtt_port);
    mqtt->set_user(mqtt_username);
    mqtt->set_password(mqtt_password);
    mqtt->set_id(hostname);
#endif

    if constexpr (use_mqtt) {
        DEBUG_PRINTLN("Setup MQTT");
        connect_wifi(hostname, ssid, password);
    }

    for (int i = 0; auto &bms : bmsArr) {
#ifdef MOCK_BATTERY
        auto battery_interface = std::make_shared<SimulatedBattery>();
        battery_interface->scenario_everything_ok();
#else
        auto battery_interface = std::make_shared<LtcMebWrapper>(i);

        switch(battery_config)
        {
            case BatteryConfig::meb12s:
                battery_interface->set_battery_type(BatteryType::meb12s);
                break;
            case BatteryConfig::meb8s:
                battery_interface->set_battery_type(BatteryType::meb8s);
                break;
            case BatteryConfig::mebAuto:
                if(!battery_interface->detect_battery())
                    battery_interface->set_battery_type(BatteryType::meb12s);
                break;
            default:
                break;
        }
#endif

        auto battery_monitor = std::make_shared<BatteryMonitor>(battery_interface);
        battery_monitor->set_battery_config(battery_config);
        bms = std::make_shared<BMS>();
        bms->set_mode(bms_mode);
        bms->set_battery_monitor(battery_monitor);

        if constexpr (use_mqtt) {
            mqtt_adapterArr[i] = std::make_shared<MqttAdapter>(bms, mqtt);

            if (i == 0) {
                mqtt_adapterArr[i]->set_ota_server(ota_server);
                mqtt_adapterArr[i]->set_ota_cert(trustRoot);
                mqtt_adapterArr[i]->init();
            } else {
                mqtt_adapterArr[i]->init(String(i));
            }
        }

        switch (bms_mode) {
            case BalanceMode::slave:
                if constexpr (use_mqtt) {
                    bms->set_balancer(mqtt_adapterArr[i]);
                }
                break;
            case BalanceMode::single:
                bms->set_balancer(std::make_shared<SingleModeBalancer>(60 * 1000, 30 * 1000));
                break;
            default:
                break;
        }

        if (i == 0) {
            auto display = std::make_shared<Display>();
            bms->set_display(display);
            display->init();
        }
        
        if (i == (bmsArr.size() - 1)) {
            bms->set_led(true);
        }

        i++;
    }

    if constexpr (use_can) {
        CAN.begin(CanBitRate::BR_500k);
    }

    DEBUG_PRINTLN("init finished");
}

int col = 0;

void loop() {
    static unsigned long lastRAMMs   = 0;
    static unsigned long lastPrintMs = 0;
    static bool          ramRequested = false;
    static bool          extRam1Requested = false;
    static bool          extRam2Requested = false;
    static bool          stateRequested  = false;
    static bool          settingsRequested = false;

    static bool writesettings = true;

    if(writesettings) {
        writesettings = false;
        vebus.writeSetting(VEBUS_SETTING_UBAT_ABSORPTION, 5000);
        while(!vebus.isSettingWriteAcked()) delay(10);
        vebus.clearSettingWriteAcked();
        vebus.writeSetting(VEBUS_SETTING_UBAT_FLOAT, 4800);
        while(!vebus.isSettingWriteAcked()) delay(10);
        vebus.clearSettingWriteAcked();
        vebus.writeSetting(VEBUS_SETTING_UBAT_LOW_LIMIT, 3700);
        while(!vebus.isSettingWriteAcked()) delay(10);
        vebus.clearSettingWriteAcked();
        vebus.writeSetting(VEBUS_SETTING_UBAT_LOW_HYSTERESIS, 400);
        while(!vebus.isSettingWriteAcked()) delay(10);
        vebus.clearSettingWriteAcked();
        vebus.writeSetting(VEBUS_SETTING_MAX_ABSORPTION_DURATION, 1);
        while(!vebus.isSettingWriteAcked()) delay(10);
        vebus.clearSettingWriteAcked();
        vebus.writeSetting(VEBUS_SETTING_CHARGE_CHARACTERISTIC, 1);
        while(!vebus.isSettingWriteAcked()) delay(10);
        vebus.clearSettingWriteAcked();
        vebus.writeSetting(VEBUS_SETTING_BATTERY_CAPACITY, 222);
        while(!vebus.isSettingWriteAcked()) delay(10);
        vebus.clearSettingWriteAcked();
        vebus.writeSetting(65, 170);
        while(!vebus.isSettingWriteAcked()) delay(10);
        vebus.clearSettingWriteAcked();
    }

    unsigned long now = millis();
    // --- Queue read RAM 500 ms before next ESS ---
    if (!settingsRequested && (now - lastPrintMs >= (PRINT_INTERVAL_MS - SETTINGS_OFFSET_MS)))
    {
        static int value = 0;

        switch (value++) {
            case 0:
                vebus.readSetting(VEBUS_SETTING_UBAT_ABSORPTION);
                //vebus.requestSettingInfo(VEBUS_SETTING_UBAT_ABSORPTION);
                break;
            case 1:
                vebus.readSetting(VEBUS_SETTING_UBAT_FLOAT);
                //vebus.requestSettingInfo(VEBUS_SETTING_UBAT_FLOAT);
                break;
            case 2:
                vebus.readSetting(VEBUS_SETTING_UBAT_LOW_LIMIT);
                //vebus.requestSettingInfo(VEBUS_SETTING_UBAT_LOW_LIMIT);
                break;
            case 3:
                vebus.readSetting(VEBUS_SETTING_UBAT_LOW_HYSTERESIS);
                //vebus.requestSettingInfo(VEBUS_SETTING_UBAT_LOW_HYSTERESIS);
                break;
            case 4:
                vebus.readSetting(VEBUS_SETTING_BATTERY_CAPACITY);
                //vebus.requestSettingInfo(VEBUS_SETTING_BATTERY_CAPACITY);
                break;
            case 5:
                vebus.readSetting(65);
                //vebus.requestSettingInfo(65);
                break;
            case 6:
                //vebus.requestRAMVarInfo(VEBUS_RAM_IBAT);
                break;
            case 7:
                //vebus.requestRAMVarInfo(VEBUS_RAM_IINVERTER_RMS);
                break;
            case 8:
                //vebus.requestRAMVarInfo(VEBUS_RAM_CHARGE_STATE);
                break;
            case 9:
                //vebus.requestRAMVarInfo(VEBUS_RAM_INVERTER_PERIOD);
                break;
            default:
                value = 0;
                break;
        }
        
        settingsRequested = true;
    }

    // --- Queue read RAM 500 ms before next ESS ---
    if (!ramRequested && (now - lastPrintMs >= (PRINT_INTERVAL_MS - RAM_OFFSET_MS)))
    {
        vebus.requestReadRAM();
        ramRequested = true;
    }

    // Queue extended RAM batch 1: mains V/A, inverter V/A, output W, mains W
    if (!extRam1Requested && (now - lastPrintMs >= EXT_RAM1_OFFSET_MS))
    {
        const uint8_t ids[] = {
            VEBUS_RAM_UMAINS_RMS, VEBUS_RAM_IMAINS_RMS,
            VEBUS_RAM_UINVERTER_RMS, VEBUS_RAM_IINVERTER_RMS,
            VEBUS_RAM_OUTPUT_POWER, VEBUS_RAM_MAINS_POWER
        };
        vebus.readRAMVars(ids, 6);
        g_lastRamBatch = 1;
        extRam1Requested = true;
    }

    // Queue extended RAM batch 2: battery current, SoC, mains freq, inverter freq
    if (!extRam2Requested && (now - lastPrintMs >= EXT_RAM2_OFFSET_MS))
    {
        const uint8_t ids[] = {
            VEBUS_RAM_IBAT, VEBUS_RAM_CHARGE_STATE,
            VEBUS_RAM_MAINS_PERIOD, VEBUS_RAM_INVERTER_PERIOD
        };
        vebus.readRAMVars(ids, 4);
        g_lastRamBatch = 2;
        extRam2Requested = true;
    }

    // Queue device state request
    if (!stateRequested && (now - lastPrintMs >= STATE_OFFSET_MS))
    {
        vebus.requestDeviceState();
        stateRequested = true;
    }

    // --- Auto-wakeup if no sync ---
    if (vebus.hasNoSync())
    {
        static unsigned long lastWakeupMs = 0;
        if (now - lastWakeupMs >= 3000)
        {
            lastWakeupMs = now;
            vebus.requestWakeup();
            Serial.println("[app] No sync — queued wakeup");
        }
    }

    // --- Check for extended RAM variable responses ---
    if (vebus.hasRAMVarResponse())
    {
        uint8_t count = vebus.getRAMVarCount();
        if (g_lastRamBatch == 1 && count >= 6)
        {
            g_mainsVoltage    = vebus.getRAMVarValue(0);
            g_mainsCurrent    = vebus.getRAMVarValue(1);
            g_inverterVoltage = vebus.getRAMVarValue(2);
            g_inverterCurrent = vebus.getRAMVarValue(3);
            g_outputPower     = vebus.getRAMVarValue(4);
            g_mainsPower      = vebus.getRAMVarValue(5);
        }
        else if (g_lastRamBatch == 2 && count >= 4)
        {
            g_batteryCurrent = vebus.getRAMVarValue(0);
            g_chargeState    = vebus.getRAMVarValue(1);
            g_mainsFreq      = vebus.getRAMVarValue(2);
            g_inverterFreq   = vebus.getRAMVarValue(3);
        }
        vebus.clearRAMVarResponse();
    }

    // --- Check for setting responses ---
    if (vebus.hasSettingResponse())
    {
        Serial.printf("[resp] Setting %d = %u (0x%04X)\n",
                      vebus.getSettingId(), vebus.getSettingValue(), vebus.getSettingValue());
        vebus.clearSettingResponse();
        settingsRequested = false;
    }

    if (vebus.isSettingWriteAcked())
    {
        Serial.println("[resp] Setting write acknowledged");
        vebus.clearSettingWriteAcked();
    }

    // --- Check for device state responses ---
    if (vebus.hasDeviceStateResponse())
    {
        uint8_t st  = vebus.getDeviceState();
        uint8_t sub = vebus.getDeviceSubState();
        Serial.printf("[resp] Device state: %s (%d)", deviceStateName(st), st);
        if (st == VEBUS_STATE_CHARGE)
            Serial.printf("  sub-state: %s (%d)", chargeSubStateName(sub), sub);
        Serial.println();
        vebus.clearDeviceStateResponse();
    }

    // --- Check for version responses ---
    if (vebus.hasVersionResponse())
    {
        Serial.printf("[resp] Firmware version: %u.%u (0x%04X 0x%04X)\n",
                      vebus.getVersionHigh(), vebus.getVersionLow(),
                      vebus.getVersionHigh(), vebus.getVersionLow());
        vebus.clearVersionResponse();
    }

    // --- Check for setting info responses ---
    if (vebus.hasSettingInfoResponse())
    {
        const VEBusSettingInfo &info = vebus.getSettingInfo();
        Serial.printf("[resp] Setting %d info: scale=%d offset=%d default=%u min=%u max=%u\n",
                      info.id, info.scale, info.offset,
                      info.defaultValue, info.minimum, info.maximum);
        vebus.clearSettingInfoResponse();
    }

    // --- Check for RAM var info responses ---
    if (vebus.hasRAMVarInfoResponse())
    {
        Serial.printf("[resp] RAM var %d info: scale=%d offset=%d\n",
                      vebus.getRAMVarInfoId(), vebus.getRAMVarInfoScale(),
                      vebus.getRAMVarInfoOffset());
        vebus.clearRAMVarInfoResponse();
        settingsRequested = false;
    }

    // --- Print status every 2 s ---
    if (now - lastPrintMs >= PRINT_INTERVAL_MS)
    {
        lastPrintMs = now;
        ramRequested = false;
        extRam1Requested = false;
        extRam2Requested = false;
        stateRequested   = false;

        if (vebus.hasNewData())
            vebus.clearNewData();

        Serial.println("--- Multiplus status ---");
        Serial.printf("  Battery voltage : %.2f V\n",  vebus.getBatVolt());
        Serial.printf("  AC power        : %d W\n",    vebus.getACPower());
        Serial.printf("  DC current      : %.1f A\n",  vebus.getDCCurrent());
        Serial.printf("  Temperature     : %.1f C\n",  vebus.getTemp());
        //Serial.printf("  ESS setpoint    : %d W\n",    (int)g_essPower);

        Serial.println("--- Multiplus power data ---");
        Serial.printf("  AC Voltage        : %d V\n",    g_inverterVoltage);
        Serial.printf("  AC current        : %d A\n",    g_inverterCurrent);
        Serial.printf("  AC power          : %d W\n",    g_outputPower);
        Serial.printf("  AC Freq           : %d Hz\n",    10.0f / (float)g_inverterFreq);
        Serial.printf("  Battery current   : %d A\n",    g_batteryCurrent);
        Serial.printf("  SOC               : %d %\n",    g_chargeState);

        byte leds = vebus.getLEDon();
        Serial.printf("  LEDs on/blink   : 0x%02X / 0x%02X", leds, vebus.getLEDblink());
        if (leds & VEBUS_LED_MAINS_ON)    Serial.print("  [MAINS]");
        if (leds & VEBUS_LED_BULK)        Serial.print("  [BULK]");
        if (leds & VEBUS_LED_ABSORPTION)  Serial.print("  [ABS]");
        if (leds & VEBUS_LED_FLOAT)       Serial.print("  [FLOAT]");
        if (leds & VEBUS_LED_INVERTER_ON) Serial.print("  [INV]");
        if (leds & VEBUS_LED_OVERLOAD)    Serial.print("  [OVERLOAD!]");
        if (leds & VEBUS_LED_LOW_BATTERY) Serial.print("  [LOW BAT!]");
        Serial.println();

        Serial.printf("  AC input limits : min=%.1f A  max=%.1f A  act=%.1f A\n",
                      vebus.getMinInputCurrentLimit(),
                      vebus.getMaxInputCurrentLimit(),
                      vebus.getActInputCurrentLimit());

        byte sw = vebus.getSwitchRegister();
        if (sw) {
            Serial.printf("  Switch register : 0x%02X", sw);
            if (sw & VEBUS_SWITCH_CHARGE)           Serial.print(" [CHG]");
            if (sw & VEBUS_SWITCH_INVERT)           Serial.print(" [INV]");
            if (sw & VEBUS_SWITCH_FRONT_UP)         Serial.print(" [FrontUP]");
            if (sw & VEBUS_SWITCH_FRONT_DOWN)       Serial.print(" [FrontDN]");
            if (sw & VEBUS_SWITCH_REMOTE_GENERATOR) Serial.print(" [GEN]");
            Serial.println();
        }

        if (vebus.getChecksumFaults() > 0)
            Serial.printf("  Checksum faults : %lu\n", vebus.getChecksumFaults());

        Serial.println();
    }

    return;

    CanMsg msg;
    unsigned long last_send = millis();

    if constexpr (use_mqtt) {
        for (auto &mqtt_adapter : mqtt_adapterArr)
            if (mqtt_adapter)
                mqtt_adapter->loop();
    } else {
        for (auto &bms : bmsArr)
            if (bms)
                bms->loop();
    }

    if constexpr (use_can) {
        if (millis() - last_send > 1000) {
            last_send = millis();
            msg.id = CanStandardId(0x01);
            msg.data_length = 4;
            msg.data[0] = 0x23;
            msg.data[1] = 0x45;
            msg.data[2] = 0x67;
            msg.data[3] = 0x89;
            CAN.write(msg);
        }
    }

    delay(100);
}
