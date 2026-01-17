/**
 * @file      GPS_MqttsPublish.ino
 * @author    Alfred Shum
 * @license   MIT
 * @copyright Copyright (c) 2026 Alfred Shum
 * @date      2026-01-17
 * @note      
 * * ESP32 code for LilyGO T-SIM7600G
 * * Connects to an MQTT Broker over TLS and publishes GPS coordinates every minute.
 * * Code based on LilyGO example and TinyGSM library from <https://github.com/Xinyuan-LilyGO/LilyGo-Modem-Series.git>
 * * Example uses a forked TinyGSM <https://github.com/lewisxhe/TinyGSM>, which will not compile successfully using the mainline TinyGSM.
 */
#define TINY_GSM_RX_BUFFER          1024 // Set RX buffer to 1Kb

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#include "utilities.h"
#include <TinyGsmClient.h>
#include <FS.h>
#include <SPIFFS.h>
#include <esp32-hal-adc.h>

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// It depends on the operator whether to set up an APN. If some operators do not set up an APN,
// they will be rejected when registering for the network. You need to ask the local operator for the specific APN.
// APNs from other operators are welcome to submit PRs for filling.
//#define NETWORK_APN     "ctlte"             //ctlte: China Telecom

// When using an IPv6 access point, the correct IPv6 APN must be configured.
bool use_ipv6_access_point = false; // Whether to use IPv6 to set the access point

#ifdef NETWORK_APN
String apn = NETWORK_APN;
#else
String apn = "";
#endif


// MQTT details
String broker = "";
uint16_t broker_port = 0;
String broker_username = "";
String broker_password = "";
String root_ca = "";
const char *client_id = "SIM7600-GPS";

const char *command_topic = "Sim7600/command";
const char *gps_topic = "Sim7600/gps";
const char *availability_topic = "Sim7600/availability";
const char *status_topic = "Sim7600/status";

const int led_pin = 12;

// Current connection index, range 0~1
const uint8_t mqtt_client_id = 0;
uint32_t next_publish_millis = 0;
const uint32_t publish_interval = 60000UL; // 1 minute
bool has_fix = false;
bool config_ok = false;
bool at_mode_notified = false;

bool publish_status();
bool load_config();

void mqtt_callback(const char *topic, const uint8_t *payload, uint32_t len)
{
    Serial.println();
    Serial.println("======mqtt_callback======");
    Serial.print("Topic:"); Serial.println(topic);
    Serial.print("Payload (hex):");
    for (uint32_t i = 0; i < len; ++i) {
        Serial.print(payload[i], HEX); Serial.print(",");
    }
    Serial.println();
    // Convert payload to string for command handling
    char buf[64] = {0};
    uint32_t copy_len = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, payload, copy_len);
    String msg = String(buf);
    msg.trim();
    msg.toLowerCase();
    if (strcmp(topic, command_topic) == 0) {
        if (msg == "on") {
            digitalWrite(led_pin, HIGH);
            Serial.println("LED -> ON");
        } else if (msg == "off") {
            digitalWrite(led_pin, LOW);
            Serial.println("LED -> OFF");
        } else if (msg == "status") {
            publish_status();
        }
    }
    Serial.println("=========================");
}

bool mqtt_connect()
{
    if (broker.isEmpty() || broker_port == 0 || root_ca.isEmpty()) {
        Serial.println("MQTT config/CA not loaded; cannot connect.");
        return false;
    }

    Serial.print("Connecting to ");
    Serial.print(broker);

    // Set will topic and message
    modem.setWillMessage(availability_topic, "offline", 1);

    bool ret = modem.mqtt_connect(mqtt_client_id, broker.c_str(), broker_port, client_id, broker_username.c_str(), broker_password.c_str());
    if (!ret) {
        Serial.println("Failed!"); return false;
    }
    Serial.println(" successfully.");

    if (!modem.mqtt_connected()) {
        return false;
    }
    // Set MQTT processing callback
    modem.mqtt_set_callback(mqtt_callback);
    // Subscribe to topic
    modem.mqtt_subscribe(mqtt_client_id, command_topic);

    return true;
}

String build_timestamp(int year, int month, int day, int hour, int minute, int second)
{
    char buf[24] = {0};
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ", year, month, day, hour, minute, second);
    return String(buf);
}

bool load_config()
{
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return false;
    }
    File file = SPIFFS.open("/config.ini", "r");
    if (!file) {
        Serial.println("Failed to open /config.ini");
        return false;
    }

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line.startsWith("#")) {
            continue;
        }
        int eq = line.indexOf('=');
        if (eq < 0) {
            continue;
        }
        String key = line.substring(0, eq);
        String value = line.substring(eq + 1);
        key.trim();
        value.trim();

        if (key == "mqtt_server") {
            broker = value;
        } else if (key == "mqtt_port") {
            broker_port = value.toInt();
        } else if (key == "mqtt_username") {
            broker_username = value;
        } else if (key == "mqtt_password") {
            broker_password = value;
        }
    }
    file.close();

    if (broker_port == 0) {
        broker_port = 8883;
    }

    Serial.print("Config loaded. Broker: "); Serial.print(broker);
    Serial.print(" Port: "); Serial.print(broker_port);
    Serial.print(" Username: "); Serial.println(broker_username);

    File cafile = SPIFFS.open("/ca.cer", "r");
    if (!cafile) {
        Serial.println("Failed to open /ca.cer");
        return false;
    }
    root_ca = cafile.readString();
    cafile.close();

    root_ca.trim();
    if (!root_ca.startsWith("-----BEGIN CERTIFICATE-----")) {
        Serial.println("CA file does not look like a PEM certificate");
        return false;
    }
    Serial.println("CA certificate loaded from SPIFFS.");

    return !broker.isEmpty() && !root_ca.isEmpty();
}

bool publish_status()
{
#ifndef BOARD_BAT_ADC_PIN
    Serial.println("Battery ADC pin not defined; cannot publish status.");
    return false;
#else
    uint32_t battery_mv = analogReadMilliVolts(BOARD_BAT_ADC_PIN);
    battery_mv *= 2; // voltage divider halves the measured voltage
#endif

#ifdef BOARD_SOLAR_ADC_PIN
    uint32_t solar_mv = analogReadMilliVolts(BOARD_SOLAR_ADC_PIN);
    solar_mv *= 2;
#endif

#ifdef MODEM_CONNECTED_ADC_PIN
    uint16_t modem_mv = modem.getBattVoltage();
#endif

    String payload = "{";
    payload += "\"battery_mv\":" + String(battery_mv);
#ifdef BOARD_SOLAR_ADC_PIN
    payload += ",\"solar_mv\":" + String(solar_mv);
#endif
#ifdef MODEM_CONNECTED_ADC_PIN
    payload += ",\"modem_mv\":" + String(modem_mv);
#endif
    payload += "}";

    Serial.print("Publishing status: ");
    Serial.println(payload);
    return modem.mqtt_publish(mqtt_client_id, status_topic, payload.c_str());
}

bool publish_location()
{
    float lat      = 0;
    float lon      = 0;
    float speed    = 0;
    float alt      = 0;
    int   vsat     = 0;
    int   usat     = 0;
    float accuracy = 0;
    int   year     = 0;
    int   month    = 0;
    int   day      = 0;
    int   hour     = 0;
    int   min      = 0;
    int   sec      = 0;
    uint8_t fixMode = 0;

    Serial.println("Requesting current GPS/GNSS/GLONASS location");
    if (!modem.getGPS(&fixMode, &lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                      &year, &month, &day, &hour, &min, &sec)) {
        Serial.println("Couldn't get GPS/GNSS/GLONASS location, will retry.");
        if (has_fix) {
            // Lost GPS fix while previously online
            modem.mqtt_publish(mqtt_client_id, availability_topic, "offline");
            has_fix = false;
        }
    return false;
    }

    String payload = "{";
    payload += "\"latitude\":" + String(lat, 6) + ",";
    payload += "\"longitude\":" + String(lon, 6) + ",";
    payload += "\"altitude\":" + String(alt, 2) + ",";
    payload += "\"speed\":" + String(speed, 2) + ",";
    payload += "\"gps_accuracy\":" + String(accuracy, 2) + ",";
//    payload += "\"fixMode\":" + String(fixMode) + ",";
//    payload += "\"visibleSatellites\":" + String(vsat) + ",";
    payload += "\"timestamp\":\"" + build_timestamp(year, month, day, hour, min, sec) + "\"";
    payload += "}";

    Serial.print("Publishing location: ");
    Serial.println(payload);
    if (!has_fix) {
        // Transition to online when first fix obtained after offline
        modem.mqtt_publish(mqtt_client_id, availability_topic, "online");
        has_fix = true;
    }
    return modem.mqtt_publish(mqtt_client_id, gps_topic, payload.c_str());
}

void setup()
{
    Serial.begin(115200); // Set console baud rate

    Serial.println("Start Sketch");

    config_ok = load_config();
    if (!config_ok) {
        Serial.println("Config/CA not loaded, continuing to AT command mode fallback.");
    }

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
#ifdef BOARD_BAT_ADC_PIN
    analogSetAttenuation(ADC_11db);
    analogReadResolution(12);
#if CONFIG_IDF_TARGET_ESP32
    analogSetWidth(12);
#endif
#endif

#ifdef BOARD_POWERON_PIN
    /* Set Power control pin output
    * * @note      Known issues, ESP32 (V1.2) version of T-A7670, T-A7608,
    *            when using battery power supply mode, BOARD_POWERON_PIN (IO12) must be set to high level after esp32 starts, otherwise a reset will occur.
    * */
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

    // Set modem reset pin ,reset modem
#ifdef MODEM_RESET_PIN
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif

#ifdef MODEM_FLIGHT_PIN
    // If there is an airplane mode control, you need to exit airplane mode
    pinMode(MODEM_FLIGHT_PIN, OUTPUT);
    digitalWrite(MODEM_FLIGHT_PIN, HIGH);
#endif

#ifdef MODEM_DTR_PIN
    // Pull down DTR to ensure the modem is not in sleep state
    Serial.printf("Set DTR pin %d LOW\n", MODEM_DTR_PIN);
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);
#endif

    // Turn on the modem
    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(MODEM_POWERON_PULSE_WIDTH_MS);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    // Check if the modem is online
    Serial.println("Start modem...");

    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.println(".");
        if (retry++ > 30) {
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(MODEM_POWERON_PULSE_WIDTH_MS);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
        }
    }
    Serial.println();

    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY) {
        sim = modem.getSimStatus();
        switch (sim) {
        case SIM_READY:
            Serial.println("SIM card online");
            break;
        case SIM_LOCKED:
            Serial.println("The SIM card is locked. Please unlock the SIM card first.");
            // const char *SIMCARD_PIN_CODE = "123456";
            // modem.simUnlock(SIMCARD_PIN_CODE);
            break;
        default:
            break;
        }
        delay(1000);
    }

#ifdef TINY_GSM_MODEM_HAS_NETWORK_MODE
    if (!modem.setNetworkMode(MODEM_NETWORK_AUTO)) {
        Serial.println("Set network mode failed!");
    }
    String mode = modem.getNetworkModeString();
    Serial.print("Current network mode : ");
    Serial.println(mode);
#endif

#ifdef TINY_GSM_MODEM_HAS_PREFERRED_MODE
    if (!modem.setPreferredMode(MODEM_PREFERRED_CATM_NBIOT)) {
        Serial.println("Set network preferred failed!");
    }
    String prefMode = modem.getPreferredModeString();
    Serial.print("Current preferred mode : ");
    Serial.println(prefMode);
#endif



#ifdef NETWORK_APN
    Serial.printf("Set network apn : %s\n", NETWORK_APN);
    if (!modem.setNetworkAPN(NETWORK_APN)) {
        Serial.println("Set network apn error !");
    }
#endif


    // Check network registration status and network signal status
    int16_t sq ;
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) {
        status = modem.getRegistrationStatus();
        switch (status) {
        case REG_UNREGISTERED:
        case REG_SEARCHING:
            sq = modem.getSignalQuality();
            Serial.printf("[%lu] Signal Quality:%d\n", millis() / 1000, sq);
            delay(1000);
            break;
        case REG_DENIED:
            Serial.println("Network registration was rejected, please check if the APN is correct");
            return ;
        case REG_OK_HOME:
            Serial.println("Online registration successful");
            break;
        case REG_OK_ROAMING:
            Serial.println("Network registration successful, currently in roaming mode");
            break;
        default:
            Serial.printf("Registration Status:%d\n", status);
            delay(1000);
            break;
        }
    }
    Serial.println();

#ifdef MODEM_REG_SMS_ONLY
    while (status == REG_SMS_ONLY) {
        Serial.println("Registered for \"SMS only\", home network (applicable only when E-UTRAN), this type of registration cannot access the network. Please check the APN settings and ask the operator for the correct APN information and the balance and package of the SIM card. If you still cannot connect, please replace the SIM card and test again. Related ISSUE: https://github.com/Xinyuan-LilyGO/LilyGo-T-A76XX/issues/307#issuecomment-3034800353");
        delay(5000);
    }
#endif

    Serial.printf("Registration Status:%d\n", status);
    delay(1000);

    String ueInfo;
    if (modem.getSystemInformation(ueInfo)) {
        Serial.print("Inquiring UE system information:");
        Serial.println(ueInfo);
    }

    /**
     *  Configure the network APN and specify whether to access the network using IPv6. If unsure, please consult your SIM card provider.
     */
    Serial.print("Connecting to network with APN:"); Serial.println(apn);
    Serial.print("Use IPv6 access point:"); Serial.println(use_ipv6_access_point ? "true" : "false");
    retry = 3;
    while (retry--) {
        if (modem.setNetworkActive(apn, use_ipv6_access_point)) {
            break;
        }
        Serial.println("Enable network failed, retry after 3s...");
        delay(3000);
    }
    if (retry < 0) {
        Serial.println("Failed to enable network!");
        return;
    }

    delay(5000);

    String ipAddress = modem.getLocalIP();
    Serial.print("Network IP:"); Serial.println(ipAddress);


    // Print modem software version
    String res;
    modem.sendAT("+SIMCOMATI");
    modem.waitResponse(10000UL, res);
    Serial.println(res);

    Serial.println("Enabling GPS/GNSS/GLONASS");
    while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
        Serial.print(".");
    }
    Serial.println();
    Serial.println("GPS Enabled");
    modem.setGPSBaud(115200);

    // Initialize MQTT, use SSL, skip authentication server
    modem.mqtt_begin(true);

    // Set SSL Certificates
    modem.mqtt_set_certificate(root_ca.c_str());

    // Connecting to MQTT Broker
    if (!mqtt_connect()) {
        Serial.println("Please make sure you are using the latest released version of the firmware. Find the latest version here: https://github.com/Xinyuan-LilyGO/LilyGo-Modem-Series#modem-firmware-upgrade-guide");
        Serial.println("If you still have problems with the latest firmware, please open an issue. Otherwise, please do not create meaningless issues.");
        return ;
    }

    // Initial availability state
    modem.mqtt_publish(mqtt_client_id, availability_topic, "offline");
    next_publish_millis = millis();
}

void loop()
{
    if (!config_ok) {
        if (!at_mode_notified) {
            Serial.println("AT command mode: config/CA not loaded. Enter AT commands.");
            at_mode_notified = true;
        }
        if (SerialAT.available()) {
            Serial.write(SerialAT.read());
        }
        if (Serial.available()) {
            SerialAT.write(Serial.read());
        }
        delay(1);
        return;
    }

    if (!modem.mqtt_connected()) {
        mqtt_connect();
    }

    if (millis() > next_publish_millis) {
        if (publish_location()) {
            next_publish_millis = millis() + publish_interval;
        } else {
            // If no fix, retry faster
            next_publish_millis = millis() + 15000UL;
        }
    }

    // MQTT handling
    modem.mqtt_handle();
    delay(5);
}

#ifndef TINY_GSM_FORK_LIBRARY
#error "No correct definition detected, Please copy all the [lib directories](https://github.com/Xinyuan-LilyGO/LilyGo-T-A76XX/tree/main/lib) to the arduino libraries directory , See README"
#endif

/*

SIM7600 Version OK 20250709
AT+SIMCOMATI
Manufacturer: SIMCOM INCORPORATED
Model: SIMCOM_SIM7600G-H
Revision: LE20B04SIM7600G22
QCN:
IMEI: xxxxxxxxxxxx
MEID:
+GCAP: +CGSM
DeviceInfo: 173,170

*/
