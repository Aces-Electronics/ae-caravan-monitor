#include <Arduino.h> // Needed for Plaformio
#include <esp_task_wdt.h>

#ifdef __cplusplus
extern "C"
{
#endif
  uint8_t temprature_sens_read(); // some hook that allows us to read the CPU core temp (probably magic?)
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

#define FORMAT_LITTLEFS_IF_FAILED true // this catches the error that occurs when you create the FS for the first time

//////// BOARD DEFINITIONS
//// uncomment for the correct board version, pick one only!
#define RC4 // PCB REV V8 RC4 or RC3
// #define RC2 // PCB REV V8 RC2 // there are two of these at HVO

////// START RC4
#ifdef RC4
#define PRODUCT "AE-CM"
#define BOARD_VERSION "V8RC4"
//// Watchdog
#define WDT_TIMEOUT 30
//// File system
#define USE_LittleFS
#define SPIFFS LITTLEFS
//// GPIOs
#define RELAY_1_ON 4            // Relay 1 on
#define RELAY_2_ON 2            // Relay 2 on
#define TEMP_SENSOR_ON 16       // Temp sensor on
#define EXTERNAL_GPS_ANTENNA 34 // Goes high if there is an external GPS antenna attached
#define FAN_ON 36               // Goes high if the case fan is running
#define SYS_LED 13              // SYS LED
//// Ethernet
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT // For ESP versions without PSRAM
#define ETH_POWER_PIN 5                   // Pin# of the enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_TYPE ETH_PHY_LAN8720          // Type of the Ethernet PHY (LAN8720 or TLK110)
#define ETH_ADDR 0                        // I²C-address of Ethernet PHY (0 or 1 for LAN8720, 31 for TLK110)
#define ETH_MDC_PIN 23                    // Pin# of the I²C clock signal for the Ethernet PHY
#define ETH_MDIO_PIN 18                   // Pin# of the I²C IO signal for the Ethernet PHY
#define NRST 5                            // Pin# connected to the NRST for the Ethernet PHY
// I2C bus
#define I2C_SCL 32 // I2C SCL
#define I2C_SDA 33 // I2C SDA
// Voltage divider
#define VOLTAGE_DIVIDER 3.92 / (158.00 + 3.92); // 158K in series with 3.92k
// ADC
#define VSPI_MISO 35 // VSPI MISO
#define VSPI_MOSI 15 // VSPI MOSI
#define VSPI_SCLK 14 // VSPI SCLK
#define VSPI_SS 12   // VSPI SS- NOTE: This does not work in the SPI set up function, worked around below
#define CS 12        // GPIO for SS workaround
// Multi reset detector
#define MRD_TIMES 2
#define ESP_MRD_USE_LITTLEFS true
#define MRD_TIMEOUT 5 // DubleResetDetector timeout in seconds
#define MRD_ADDRESS 0 // RTC Memory Address for the MultiResetDetector to use
// Adafruit IO details
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "84ace"
#define AIO_KEY "5dd3d693bdb54f048e43c7e901208521"
#define AIO_FEED_NAME_B "/feeds/boss-aecc-b"
#define AIO_FEED_NAME_S "/feeds/boss-aecc-s"
#define AIO_FEED_NAME_C "/feeds/boss-aecc-c"
#define AIO_FEED_NAME_T "/feeds/boss-aecc-t"
#define MQTT_KEEP_ALIVE 300
#endif
////// END RC4

//////// END board definitions

//////// Import libraries
#include <ElegantOTA.h>                       // OTA
#include <ESPmDNS.h>                          // MDNS
#include <ESP_MultiResetDetector.h>           // Multi reset detector https://github.com/khoih-prog/ESP_MultiResetDetector
#include <ETH.h>                              // Ethernet
#include <FS.h>                               // ESP FS
#include <Preferences.h>                      // Use flash to save data between restarts
#include <SPI.h>                              // SPI for ADC
#include <SimpleTimer.h>                      // Interrupt based timer
#include "SparkFun_I2C_GPS_Arduino_Library.h" // I2C GPS
#include <TinyGPS++.h>                        // I2C GPS
#include "uptime.h"                           // Uptime library
#include <WebSerial.h>                        // Webserial
#include <Wire.h>                             // I2C GPS
#include <WiFi.h>                             // WiFi
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
////// End import required libraries

//////// Initialise all the things
String prod_release = "False";                               // stores the static IP state ("True" = static, "False" = DHPC)
String ae_cm_full_version = PRODUCT;                         // stores the ae-cm version
String ae_cm_version = BOARD_VERSION;                        // stores the board version
String compile_date = (String(__DATE__).c_str());            // stores the compile-time date
String compile_time = (String(__TIME__).c_str());            // stores the compile-time time
String firmware_version = compile_date + " " + compile_time; // creates the firmware_version

String http_username = "admin"; // HTTP username
String http_password = "admin"; // HTTP password + UDP OTA upload password

String udp_payload_ip = "192.168.137.1"; // UDP payload, send to IP address- TODO: use this
unsigned int udp_payload_p = 2331;       // UDP payload, send to port number- TODO: use this

bool eth_enabled = 0;    // turn on Ethernet Mode
bool ap_enabled = 0;     // turns on WiFi AP Mode
bool sta_enabled = 0;    // turn on STA Mode
bool udp_payload_en = 0; // enables the UDP payload sender (to IMS), same info as at /api
bool settings_saved = false;

String gps_antenna = "CHECKING";   // stores the GPS antenna state
String enclosure_fan = "CHECKING"; // stores the enclosure fan state
String relay1_state = "OFF";       // stores relay1 state data initially
String relay2_state = "OFF";       // stores relay2 state data initially

int minHDOPAllowed = 3; // minimum allowed GPS HDOP to consider location valid- TODO: use this and also implement some movement hysteresis
char gpsLAT[20] = "0.0000";
char gpsLON[20] = "0.0000";

String hostname;         // stores the hostname
String friendly_name;    // stores the label for the device
String friendly_product; // stores the label for the device
String friendly_version; // stores the label for the device
String static_ip_en;     // stores a static IP is enabled
String ipaddressL;       // stores the static IPv4 address
String subnetL;          // stores the static IPv4 subnet
String gatewayL;         // stores the static IPv4 gateway
String dnsL;             // stores the IPv4 dns server address
String ch0_name;         // stores the name of the channel
String ch1_name;         // stores the ADC CH 1 friendly name
String ch2_name;         // stores the ADC CH 1 friendly name
String ch3_name;         // stores the ADC CH 1 friendly name
String relay1_name;      // stores relay1 friendly name
String relay2_name;      // stores relay2 friendly name
String ap_ip_address;    // stores the wifi ip when in AP mode
String ap_subnet;        // stores the wifi subnet when in AP mode
String ap_gateway;       // stores the wifi gateway when in AP mode
String sta_ip_address;   // stores the sta ip when in STA mode
String sta_subnet;       // stores the sta subnet when in STA mode
String sta_gateway;      // stores the sta gateway when in STA mode
String sta_dnsL;         // stores the sta dns when in STA mode
String api_string;       // stores the API data that becomes available at /api

String sta_ssid;         // STA SSID placeholder
String sta_password;     // STA password placeholder

String ap_ssid;          // AP SSID placeholder
String ap_password;      // AP password placeholder

String warnings0 = "";   // a place to store a warning about a subsystem or event
String warnings1 = "";   // a place to store a warning about a subsystem or event
String warnings2 = "";   // a place to store a warning about a subsystem or event
String warnings3 = "";   // a place to store a warning about a subsystem or event

bool firstBoot = true;             // sets the loop() to firstboot mode
bool dhcpError = false;            // placeholder for DHCP error flag
bool saveOnly = false;             // allows the syncsettings function to skip the read before write (ignores corrupt data
bool MRD_Detected = false;         // MRD
static bool eth_connected = false; // set the ethernet state as disconnected at boot
static const int spiClk = 1000000; // 1 MHz for ADC SPI
unsigned long ota_progress_millis = 0;

String utc_date; // stores GPS UTC date
String utc_time; // stores GPS UTC time
String iso_date; // stores an string for iso8601 formatting
String iso_time; // stores an string for iso8601 formatting

SimpleTimer logTimer;            // stores log timer data
SimpleTimer getSensorDataTimer;  // stores getSensorData timer data
SimpleTimer updateMDNSDataTimer; // stores updateMDNSData timer data

// ADC variables
const byte channel_0 = 0x18;                   // ADC channel register
const byte channel_1 = 0x19;                   // ADC channel register
const byte channel_2 = 0x1A;                   // ADC channel register
const byte channel_3 = 0x1B;                   // ADC channel register
const int ADC_resolution = 4095;               // 12 bits ADC
const float voltage_divider = VOLTAGE_DIVIDER; // Voltage divider as per board version
const float voltage_reference = 2.495;         // ? Vincenzo?
uint16_t reading;                              // Storage for ADC reading

byte byte0; // highbyte for bit11-8
byte byte1; // lowbyte for bit7-0

float ch0_temperature; // VIn & temperature. Temp unless TEMP_SENSOR_ON is LOW
float ch0_voltage;     // VIn & temperature. VIn unless TEMP_SENSOR_ON is HIGH
float ch1_voltage;     // ADC Spare 1
float ch2_voltage;     // ADC Spare 2
float ch3_voltage;     // ADC Spare 3

unsigned long previousTime = 0;

String uptimeString = "0d:0h:0m:0s"; // placeholder for uptime string

const char *input_parameter1 = "hostname"; // URL encoded strangs to match against
const char *input_parameter2 = "friendlyname";
const char *input_parameter3 = "ipaddress";
const char *input_parameter4 = "subnet";
const char *input_parameter5 = "gateway";
const char *input_parameter6 = "dns";
const char *input_parameter7 = "ch1name";
const char *input_parameter8 = "ch2name";
const char *input_parameter9 = "ch3name";
const char *input_parameter10 = "relay1name";
const char *input_parameter11 = "relay2name";
const char *input_parameter12 = "staticethipen";
const char *input_parameter13 = "httpusername";
const char *input_parameter14 = "httppassword";
const char *input_parameter15 = "stassid";
const char *input_parameter16 = "stapwd";
const char *input_parameter17 = "staenabled";
const char *input_parameter18 = "apssid";
const char *input_parameter19 = "appwd";
const char *input_parameter20 = "apenabled";
const char *input_parameter21 = "udppayloadip";
const char *input_parameter22 = "udppayloadp";
const char *input_parameter23 = "apipaddress";
const char *input_parameter24 = "apsubnet";
const char *input_parameter25 = "apgateway";
const char *input_parameter26 = "staipaddress";
const char *input_parameter27 = "stasubnet";
const char *input_parameter28 = "stagateway";
const char *input_parameter29 = "udppayloaden";
const char *input_parameter30 = "_";
const char *input_parameter31 = "ethenabled";

unsigned long lastTime = 0;       // needed for the web server send events
unsigned long timerDelay = 30000; // needed for the web server send events

// Watchdog
int i = 0;
int last = millis();
int BootReason = 99;

IPAddress remoteIP = remoteIP.fromString(udp_payload_ip); // create the IP byte array from the strang

SPIClass *vspi = NULL; // stores the vspi data

// Adafruit MQTT
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish BOSS_CM_B = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED_NAME_B); // battery v
Adafruit_MQTT_Publish BOSS_CM_S = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED_NAME_S); // solar v
Adafruit_MQTT_Publish BOSS_CM_C = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED_NAME_C); // car v
Adafruit_MQTT_Publish BOSS_CM_T = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED_NAME_T); // Temp

Preferences preferences;            // Hook the object to the library
I2CGPS myI2CGPS;                    // Hook object to the library
TinyGPSPlus gps;                    // Hook object to the library
AsyncWebServer server(80);          // Hook object to the library
AsyncEventSource events("/events"); // Hook object to the library
MultiResetDetector *mrd;            // Hook object to the library
WiFiUDP Udp;                        // Hook object to the library
File root;
//// End initialise all the things

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  WebSerial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    WebSerial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
    WebSerial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
    WebSerial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void calculateUptime()
{ // calculate the hardware uptime
  uptime::calculateUptime();
  uptimeString = "";

  Serial.print("Uptime: ");
  Serial.print(uptime::getDays());
  Serial.print(" days: ");
  Serial.print(uptime::getHours());
  Serial.print(" hours: ");
  Serial.print(uptime::getMinutes());
  Serial.print(" minutes: ");
  Serial.print(uptime::getSeconds());
  Serial.println(" seconds");

  uptimeString = (String)uptime::getDays() + "d:" + uptime::getHours() + "h:" + uptime::getMinutes() + "m:" + uptime::getSeconds() + "s";
  events.send(String(uptimeString).c_str(), "uptime", millis()); // update the web page
  WebSerial.print("Uptime: ");                                   // webserial
  WebSerial.println(uptimeString);
}

void MQTT_connect()
{
  int8_t again;

  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  if (mqtt.connected())
  {
    return;
  }

  Serial.println("Connecting to Adafruit IO...");
  uint8_t retry = 5;

  if ((again = mqtt.connect()) != 0)
  {
    Serial.println(mqtt.connectErrorString(again));
    mqtt.disconnect();
  }
}

void stopWIFI()
{
  WiFi.mode(WIFI_MODE_NULL); // unsets the WiFi mode
  WiFi.mode(WIFI_OFF);       // shuts down the WiFi radio
  btStop();                  // shuts down the ble radio which given the above line will actually shutdown the multiplexed radio (and save power)
}

void updateMDNSData()
{ // stop the MDNS broadcaster, update the data, start it again, also, kill WiFi if it isn't needed
  MDNS.end();

  Serial.println("Updating MDNS data");
  WebSerial.println("Updating MDNS data");

  MDNS.begin(String(hostname).c_str());
  MDNS.addService("_aecm", "_tcp", 80);
  MDNS.addServiceTxt("_aecm", "_tcp", "type", ae_cm_full_version.c_str());
  MDNS.addServiceTxt("_aecm", "_tcp", "id", WiFi.macAddress());
  MDNS.addServiceTxt("_aecm", "_tcp", "webserver port", "80");
  MDNS.addServiceTxt("_aecm", "_tcp", "info", "AE CM, voltage monitor with relay/device control");
  MDNS.addServiceTxt("_aecm", "_tcp", "uptime", uptimeString.c_str());
  MDNS.addServiceTxt("_aecm", "_tcp", "temperature", (String(ch0_temperature).c_str()));
}

void check_status()
{
  static ulong checkstatus_timeout = 0;
  static ulong current_millis;
#define MRD_CHECK_INTERVAL 100L
  current_millis = millis();

  // If MRD_Detected, don't need to blink
  if (!MRD_Detected && ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0)))
  {
    checkstatus_timeout = current_millis + MRD_CHECK_INTERVAL;
  }
}

void syncSettings()
{ // read back all the data in flash on boot
  preferences.begin("settings", false);
  if ((preferences.getString("hostname") != "") && (!saveOnly))
  {
    Serial.println("Reading back saved settings from Flash, shouldn't see this on MRD!: ");
    hostname = preferences.getString("hostname");
    friendly_name = preferences.getString("friendly_name");
    static_ip_en = preferences.getString("static_ip_en");
    ipaddressL = preferences.getString("ip_address");
    subnetL = preferences.getString("subnet");
    gatewayL = preferences.getString("gateway");
    dnsL = preferences.getString("dns");
    ch1_name = preferences.getString("ch1_name");
    ch2_name = preferences.getString("ch2_name");
    ch3_name = preferences.getString("ch3_name");
    relay1_name = preferences.getString("relay1_name");
    relay2_name = preferences.getString("relay2_name");
    http_username = preferences.getString("http_username");
    http_password = preferences.getString("http_password");
    ap_enabled = preferences.getBool("ap_enabled");
    ap_ssid = preferences.getString("ap_ssid");
    ap_password = preferences.getString("ap_password").c_str();
    ap_ip_address = preferences.getString("ap_ip_address");
    ap_subnet = preferences.getString("ap_subnet");
    ap_gateway = preferences.getString("ap_gateway");
    sta_enabled = preferences.getBool("sta_enabled");
    sta_ssid = preferences.getString("sta_ssid");
    sta_password = preferences.getString("sta_password").c_str();
    sta_ip_address = preferences.getString("sta_ip_address");
    sta_subnet = preferences.getString("sta_subnet");
    sta_gateway = preferences.getString("sta_gateway");
    sta_dnsL = preferences.getString("sta_dns");
    udp_payload_en = preferences.getBool("udp_payload_en");
    udp_payload_ip = preferences.getString("udp_payload_ip");
    udp_payload_p = preferences.getInt("udp_payload_p");
    eth_enabled = preferences.getBool("eth_enabled");

    Serial.println("The following settings have been synced from Flash, no MRD:");
    Serial.print("hostname: ");
    Serial.println(hostname);
    Serial.print("friendly_name: ");
    Serial.println(friendly_name);
    Serial.print("staticethipen: ");
    Serial.println(static_ip_en);
    Serial.print("ipaddress: ");
    Serial.println(ipaddressL);
    Serial.print("subnet: ");
    Serial.println(subnetL);
    Serial.print("gateway: ");
    Serial.println(gatewayL);
    Serial.print("dns: ");
    Serial.println(dnsL);
    Serial.print("ch1name: ");
    Serial.println(ch1_name);
    Serial.print("ch2name: ");
    Serial.println(ch2_name);
    Serial.print("ch3name: ");
    Serial.println(ch3_name);
    Serial.print("relay1name: ");
    Serial.println(relay1_name);
    Serial.print("relay2name: ");
    Serial.println(relay2_name);
    Serial.print("httpusername: ");
    Serial.println(http_username);
    Serial.print("httppassword: ");
    Serial.println(http_password);
    Serial.print("apenabled: ");
    Serial.println(ap_enabled);
    Serial.print("apssid: ");
    Serial.println(ap_ssid);
    Serial.print("appassword: ");
    Serial.println(ap_password);
    Serial.print("apipaddress: ");
    Serial.println(ap_ip_address);
    Serial.print("apsubnet: ");
    Serial.println(ap_subnet);
    Serial.print("apgateway: ");
    Serial.println(ap_gateway);
    Serial.print("staenabled: ");
    Serial.println(sta_enabled);
    Serial.print("stassid: ");
    Serial.println(sta_ssid);
    Serial.print("stapassword: ");
    Serial.println(sta_password);
    Serial.print("staipaddress: ");
    Serial.println(sta_ip_address);
    Serial.print("stasubnet: ");
    Serial.println(sta_subnet);
    Serial.print("stagateway: ");
    Serial.println(sta_gateway);
    Serial.print("stadns: ");
    Serial.println(sta_dnsL);
    Serial.print("udppayloaden: ");
    Serial.println(udp_payload_en);
    Serial.print("udppayloadip: ");
    Serial.println(udp_payload_ip);
    Serial.print("udppayloadp: ");
    Serial.println(udp_payload_p);
    Serial.print("ethenabled: ");
    Serial.println(eth_enabled);
  }
  else
  {
    Serial.println("The following settings were in flash before the MRD:");
    Serial.print("hostname: ");
    Serial.println(hostname);
    Serial.print("friendly_name: ");
    Serial.println(friendly_name);
    Serial.print("staticethipen: ");
    Serial.println(static_ip_en);
    Serial.print("ipaddress: ");
    Serial.println(ipaddressL);
    Serial.print("subnet: ");
    Serial.println(subnetL);
    Serial.print("gateway: ");
    Serial.println(gatewayL);
    Serial.print("dns: ");
    Serial.println(dnsL);
    Serial.print("ch1name: ");
    Serial.println(ch1_name);
    Serial.print("ch2name: ");
    Serial.println(ch2_name);
    Serial.print("ch3name: ");
    Serial.println(ch3_name);
    Serial.print("relay1name: ");
    Serial.println(relay1_name);
    Serial.print("relay2name: ");
    Serial.println(relay2_name);
    Serial.print("httpusername: ");
    Serial.println(http_username);
    Serial.print("httppassword: ");
    Serial.println(http_password);
    Serial.print("apenabled: ");
    Serial.println(ap_enabled);
    Serial.print("apssid: ");
    Serial.println(ap_ssid);
    Serial.print("appassword: ");
    Serial.println(ap_password);
    Serial.print("apipaddress: ");
    Serial.println(ap_ip_address);
    Serial.print("apsubnet: ");
    Serial.println(ap_subnet);
    Serial.print("apgateway: ");
    Serial.println(ap_gateway);
    Serial.print("staenabled: ");
    Serial.println(sta_enabled);
    Serial.print("stassid: ");
    Serial.println(sta_ssid);
    Serial.print("stapassword: ");
    Serial.println(sta_password);
    Serial.print("staipaddress: ");
    Serial.println(sta_ip_address);
    Serial.print("stasubnet: ");
    Serial.println(sta_subnet);
    Serial.print("stagateway: ");
    Serial.println(sta_gateway);
    Serial.print("stadns: ");
    Serial.println(sta_dnsL);
    Serial.print("udppayloaden: ");
    Serial.println(udp_payload_en);
    Serial.print("udppayloadip: ");
    Serial.println(udp_payload_ip);
    Serial.print("udppayloadp: ");
    Serial.println(udp_payload_p);
    Serial.print("ethenabled: ");
    Serial.println(eth_enabled);

    Serial.println("!!!FACTORY RESET OF ALL SETTINGS HAS OCCURED!!!");
    preferences.putString("hostname", hostname);               // add new setting to flash storage
    preferences.putString("friendly_name", friendly_name);     // add new setting to flash storage
    preferences.putString("static_ip_en", static_ip_en);       // add new setting to flash storage
    preferences.putString("ip_address", ipaddressL);           // add new setting to flash storage
    preferences.putString("subnet", subnetL);                  // add new setting to flash storage
    preferences.putString("gateway", gatewayL);                // add new setting to flash storage
    preferences.putString("dns", dnsL);                        // add new setting to flash storage
    preferences.putString("ch1_name", ch1_name);               // add new setting to flash storage
    preferences.putString("ch2_name", ch2_name);               // add new setting to flash storage
    preferences.putString("ch3_name", ch3_name);               // add new setting to flash storage
    preferences.putString("relay1_name", relay1_name);         // add new setting to flash storage
    preferences.putString("relay2_name", relay2_name);         // add new setting to flash storage
    preferences.putString("http_username", http_username);     // add new setting to flash storage
    preferences.putString("http_password", http_password);     // add new setting to flash storage
    preferences.putBool("ap_enabled", ap_enabled);             // add new setting to flash storage
    preferences.putString("ap_ssid", ap_ssid);                 // add new setting to flash storage
    preferences.putString("ap_password", ap_password);         // add new setting to flash storage
    preferences.putString("ap_ip_address", ap_ip_address);     // add new setting to flash storage
    preferences.putString("ap_subnet", ap_subnet);             // add new setting to flash storage
    preferences.putString("ap_gateway", ap_gateway);           // add new setting to flash storage
    preferences.putBool("sta_enabled", sta_enabled);         // add new setting to flash storage
    preferences.putString("sta_ssid", sta_ssid);               // add new setting to flash storage
    preferences.putString("sta_password", sta_password);       // add new setting to flash storage
    preferences.putString("sta_ip_address", sta_ip_address);   // add new setting to flash storage
    preferences.putString("sta_subnet", sta_subnet);           // add new setting to flash storage
    preferences.putString("sta_gateway", sta_gateway);         // add new setting to flash storage
    preferences.putString("sta_dns", sta_dnsL);                // add new setting to flash storage
    preferences.putBool("udp_payload_en", udp_payload_en);     // add new setting to flash storage
    preferences.putString("udp_payload_ip", udp_payload_ip);   // add new setting to flash storage
    preferences.putInt("udp_payload_p", udp_payload_p);        // add new setting to flash storage
    preferences.putInt("eth_enabled", eth_enabled);        // add new setting to flash storage
  }
  Serial.println("The following settings are now in use and are synced with flash:");
  Serial.print("hostname: ");
  Serial.println(hostname);
  Serial.print("friendly_name: ");
  Serial.println(friendly_name);
  Serial.print("staticethipen: ");
  Serial.println(static_ip_en);
  Serial.print("ipaddress: ");
  Serial.println(ipaddressL);
  Serial.print("subnet: ");
  Serial.println(subnetL);
  Serial.print("gateway: ");
  Serial.println(gatewayL);
  Serial.print("dns: ");
  Serial.println(dnsL);
  Serial.print("ch1name: ");
  Serial.println(ch1_name);
  Serial.print("ch2name: ");
  Serial.println(ch2_name);
  Serial.print("ch3name: ");
  Serial.println(ch3_name);
  Serial.print("relay1name: ");
  Serial.println(relay1_name);
  Serial.print("relay2name: ");
  Serial.println(relay2_name);
  Serial.print("httpusername: ");
  Serial.println(http_username);
  Serial.print("httppassword: ");
  Serial.println(http_password);
  Serial.print("apenabled: ");
  Serial.println(ap_enabled);
  Serial.print("apssid: ");
  Serial.println(ap_ssid);
  Serial.print("appassword: ");
  Serial.println(ap_password);
  Serial.print("apipaddress: ");
  Serial.println(ap_ip_address);
  Serial.print("apsubnet: ");
  Serial.println(ap_subnet);
  Serial.print("apgateway: ");
  Serial.println(ap_gateway);
  Serial.print("staenabled: ");
  Serial.println(sta_enabled);
  Serial.print("stassid: ");
  Serial.println(sta_ssid);
  Serial.print("stapassword: ");
  Serial.println(sta_password);
  Serial.print("staipaddress: ");
  Serial.println(sta_ip_address);
  Serial.print("stasubnet: ");
  Serial.println(sta_subnet);
  Serial.print("stagateway: ");
  Serial.println(sta_gateway);
  Serial.print("stadns: ");
  Serial.println(sta_dnsL);
  Serial.print("udppayloaden: ");
  Serial.println(udp_payload_en);
  Serial.print("udppayloadip: ");
  Serial.println(udp_payload_ip);
  Serial.print("udppayloadp: ");
  Serial.println(udp_payload_p);
  Serial.print("ethenabled: ");
  Serial.println(eth_enabled);
  Serial.println();
  preferences.end();
}

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
  if (d == "settings"){
    Serial.print("WebSerial has received: ");
    Serial.println(d);
    saveOnly = false;
    syncSettings (); // shows the settings that are in use
  }
}

String processor(const String &var)
{
  if (var == "ch0temperature")
  {
    return String(ch0_temperature);
  }
  else if (var == "ch0voltage")
  {
    return String(ch0_voltage);
  }
  else if (var == "ch1voltage")
  {
    return String(ch1_voltage);
  }
  else if (var == "ch2voltage")
  {
    return String(ch2_voltage);
  }
  else if (var == "ch3voltage")
  {
    return String(ch3_voltage);
  }
  else if (var == "ch0name")
  {
    return String(ch0_name);
  }
  else if (var == "ch1name")
  {
    return String(ch1_name);
  }
  else if (var == "ch2name")
  {
    return String(ch2_name);
  }
  else if (var == "ch3name")
  {
    return String(ch3_name);
  }
  else if (var == "relay1state")
  {
    return String(relay1_state);
  }
  else if (var == "relay2state")
  {
    return String(relay2_state);
  }
  else if (var == "relay1name")
  {
    return String(relay1_name);
  }
  else if (var == "relay2name")
  {
    return String(relay2_name);
  }
  else if (var == "lat")
  {
    return String(gps.location.lat(), 6);
  }
  else if (var == "lon")
  {
    return String(gps.location.lng(), 6);
  }
  else if (var == "alt")
  {
    return String(gps.altitude.meters());
  }
  else if (var == "sats")
  {
    return String(gps.satellites.value());
  }
  else if (var == "gpsages")
  {
    return String(gps.location.age() / 1000);
  }
  else if (var == "hdop")
  {
    return String(gps.hdop.hdop());
  }
  else if (var == "utctime")
  {
    return String(utc_time);
  }
  else if (var == "utcdate")
  {
    return String(utc_date);
  }
  else if (var == "enclosurefan")
  {
    return String(enclosure_fan);
  }
  else if (var == "gpsant")
  {
    return String(gps_antenna);
  }
  else if (var == "aecmversion") // ToDo: Fix me
  {
    return String(ae_cm_version);
  }
  else if (var == "hostname")
  {
    return String(hostname);
  }
  else if (var == "friendlyname")
  {
    return String(friendly_name);
  }
  else if (var == "staticethipen")
  {
    return String(static_ip_en);
  }
  else if (var == "ipaddress")
  {
    return String(ipaddressL);
  }
  else if (var == "subnet")
  {
    return String(subnetL);
  }
  else if (var == "gateway")
  {
    return String(gatewayL);
  }
  else if (var == "dns")
  {
    return String(dnsL);
  }
  else if (var == "cputemp")
  {
    return String(((temprature_sens_read() - 32) / 1.8));
  }
  else if (var == "uptime")
  {
    return String(uptimeString);
  }
  else if (var == "warnings0")
  {
    return String(warnings0);
  }
  else if (var == "warnings1")
  {
    return String(warnings2);
  }
  else if (var == "warnings2")
  {
    return String(warnings1);
  }
  else if (var == "warnings3")
  {
    return String(warnings3);
  }
  else if (var == "httpusername")
  {
    return String(http_username);
  }
  else if (var == "httppassword")
  {
    return String(http_password);
  }
  else if (var == "apenabled")
  {
    return String(ap_enabled);
  }
  else if (var == "apssid")
  {
    return String(ap_ssid);
  }
  else if (var == "appwd")
  {
    return String(ap_password);
  }
  else if (var == "apipaddress")
  {
    return String(ap_ip_address);
  }
  else if (var == "apsubnet")
  {
    return String(ap_subnet);
  }
  else if (var == "apgateway")
  {
    return String(ap_gateway);
  }
  else if (var == "staenabled")
  {
    return String(sta_enabled);
  }
  else if (var == "stassid")
  {
    return String(sta_ssid);
  }
  else if (var == "stapwd")
  {
    return String(sta_password);
  }
  else if (var == "staipaddress")
  {
    return String(sta_ip_address);
  }
  else if (var == "stasubnet")
  {
    return String(sta_subnet);
  }
  else if (var == "stagateway")
  {
    return String(sta_gateway);
  }
  else if (var == "_")
  {
    return String(sta_dnsL);
  }
  else if (var == "udppayloaden")
  {
    return String(udp_payload_en);
  }
  else if (var == "udppayloadip")
  {
    return String(udp_payload_ip);
  }
  else if (var == "udppayloadp")
  {
    return String(udp_payload_p);
  }
  else if (var == "ethenabled")
  {
    return String(eth_enabled);
  }
  else
  {
    return String("UNDEFINED!!!");
  }
  return String();
}

// TODO: make the below into something that lets the user know that the AE_CM couldn't connect to the wifi network of choice via the web interface
// maybe use any of these interseting events as well https://github.com/espressif/arduino-esp32/blob/af119215353b26d944b80d7126bed68e060e6697/tools/sdk/include/esp32/esp_event_legacy.h ?
void OnWiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_AP_START:
    Serial.println("ESP32 soft AP started");
    WebSerial.println("ESP32 soft AP started");
    break;
  case SYSTEM_EVENT_AP_STACONNECTED:
    Serial.println("Station connected to ESP32 soft AP");
    WebSerial.println("Station connected to ESP32 soft AP");
    break;
  case SYSTEM_EVENT_AP_STADISCONNECTED:
    Serial.println("Station disconnected from ESP32 soft AP");
    WebSerial.println("Station disconnected from ESP32 soft AP");
    break;
  default:
    break;
  }
}

// returns a 'not found' to a 404
void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

// sets up the interface used for Ethernet
void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  // Ethernet
  case SYSTEM_EVENT_ETH_START:
    Serial.println("ETH Started!");
    // set eth hostname here
    ETH.setHostname(hostname.c_str()); // take the string "hostname" and make it a string buffer
    Serial.print("Hostname: ");
    Serial.println(ETH.getHostname());
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    Serial.println("ETH Connected!");
    break;
  case SYSTEM_EVENT_ETH_GOT_IP:
    Serial.print("ETH MAC: ");
    Serial.print(ETH.macAddress());
    Serial.print(", IPv4: ");
    Serial.print(ETH.localIP());
    ipaddressL = ETH.localIP().toString();
    Serial.print(", subnet: ");
    Serial.println(ETH.subnetMask());
    subnetL = ETH.subnetMask().toString();
    Serial.print("Gateway IP: ");
    Serial.print(ETH.gatewayIP());
    gatewayL = ETH.gatewayIP().toString();
    Serial.print(", dns IP: ");
    Serial.print(ETH.dnsIP());
    dnsL = ETH.dnsIP().toString();
    if (ETH.fullDuplex())
    {
      Serial.print(", FULL_DUPLEX");
    }
    Serial.print(", ");
    Serial.print(ETH.linkSpeed());
    Serial.println("Mbps!");
    eth_connected = true;
    saveOnly = true;  // allow the function below to overwrite saved values
    syncSettings();   // synces string info to persistent storage (will overwrite static config with DHCP when DHCP is used)
    saveOnly = false; // put her back to stock for when she goes over the pits
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    Serial.println("ETH Disconnected!!!");
    eth_connected = false;
    break;
  case SYSTEM_EVENT_ETH_STOP:
    Serial.println("ETH Stopped!!!");
    eth_connected = false;
    break;
  case SYSTEM_EVENT_WIFI_READY:
    Serial.println("WiFi interface ready");
    break;
  case SYSTEM_EVENT_SCAN_DONE:
    Serial.println("Completed scan for access points");
    break;
  case SYSTEM_EVENT_AP_START:
    Serial.println("WiFi access point started");
    break;
  case SYSTEM_EVENT_AP_STOP:
    Serial.println("WiFi access point  stopped");
    break;
  case SYSTEM_EVENT_AP_STACONNECTED:
    Serial.println("Client connected");
    break;
  case SYSTEM_EVENT_AP_STADISCONNECTED:
    Serial.println("Client disconnected");
    break;
  case SYSTEM_EVENT_AP_STAIPASSIGNED:
    Serial.println("Assigned IP address to client");
    break;
  case SYSTEM_EVENT_AP_PROBEREQRECVED:
    Serial.println("Received probe request");
    break;
  case SYSTEM_EVENT_GOT_IP6:
    Serial.println("IPv6 is preferred");
    break;
    // ToDO: add the STA config
  default:
    break;
  }
}

float convert_to_voltage(int ADC_reading, int ADC_resolution, float voltage_divider, float voltage_reference)
{
  if (ADC_reading == 0)
    return 0;
  else
  {
    ADC_reading++; // ADC readings are 1 bit lower than they should be. Could be from rounding.
    return voltage_reference * (((float)(ADC_reading) / ADC_resolution) / voltage_divider);
  }
}

float convert_to_temperature(int ADC_reading, int ADC_resolution, float voltage_reference)
{
  const float volt_temp_ratio = 0.01;    // change in volts per change in degrees C =  10mV
  const float zero_degree_voltage = 0.5; // voltage when sensor is at 0 degrees C = 0.5V
  float voltage = voltage_reference * ((float)ADC_reading / ADC_resolution);
  float temperature = (voltage - zero_degree_voltage) / volt_temp_ratio;
  return temperature;
}

uint16_t transfer_data(byte channel)
{
  digitalWrite(CS, LOW);        // Need to write our own code to pull down CS. The spi transfer function doesn't work.
  vspi->transfer(channel);      // 0b0000011000 configuration bits: five 0's, startbit, sind/diff, D2,D1,D0(in this case channel 0)
  byte0 = vspi->transfer(0x00); // transfer something to request data (synchronous communication)
  byte1 = vspi->transfer(0x00); // transfer something to request data (synchronous communication)
  digitalWrite(CS, HIGH);
  return ((byte0 << 8 | byte1) >> 2) & 0xFFF; // << THIS IS THE CORRECT ONE! (Thanks Marco!)
}

void getFANState()
{
  if (digitalRead(FAN_ON))
  {
    enclosure_fan = "ON";
  }
  else
  {
    enclosure_fan = "OFF";
  }
  events.send(String(enclosure_fan).c_str(), "enclosureFan", millis()); // update the web page
}

void getGPSAntennaState()
{
  if (!digitalRead(EXTERNAL_GPS_ANTENNA))
  {
    gps_antenna = "CONNECTED";
  }
  else
  {
    gps_antenna = "DISCONNECTED";
  }
  events.send(String(gps_antenna).c_str(), "gpsant", millis()); // update the web page
}

void readFromADC()
{
  // Read temperature
  digitalWrite(TEMP_SENSOR_ON, HIGH); // Select temperature sensor on analog switch
  reading = transfer_data(channel_0);
  ch0_temperature = convert_to_temperature(reading, ADC_resolution, voltage_reference);
  events.send(String(ch0_temperature).c_str(), "ch0temperature", millis());   // update the web page
  events.send(String(firmware_version).c_str(), "firmwareversion", millis()); // update the web page

  // Read VIN
  digitalWrite(TEMP_SENSOR_ON, LOW); // Select temperature sensor on analog switch
  reading = transfer_data(channel_0);
  ch0_voltage = convert_to_voltage(reading, ADC_resolution, voltage_divider, voltage_reference);
  if (ch0_voltage < 0.1)
  {
    ch0_voltage = 0.00;
  }
  events.send(String(ch0_voltage).c_str(), "ch0voltage", millis()); // update the web page

  // Read Channel_1
  reading = transfer_data(channel_1);
  ch1_voltage = convert_to_voltage(reading, ADC_resolution, voltage_divider, voltage_reference);
  if (ch1_voltage < 0.1)
  {
    ch1_voltage = 0.00;
  }
  events.send(String(ch1_voltage).c_str(), "ch1voltage", millis()); // update the web page

  // Read Channel_2
  reading = transfer_data(channel_2);
  ch2_voltage = convert_to_voltage(reading, ADC_resolution, voltage_divider, voltage_reference);
  if (ch2_voltage < 4.0)
  {
    ch2_voltage = 0.00;
  }
  events.send(String(ch2_voltage).c_str(), "ch2voltage", millis()); // update the web page

  // Read Channel_3
  reading = transfer_data(channel_3);
  ch3_voltage = convert_to_voltage(reading, ADC_resolution, voltage_divider, voltage_reference);
  if (ch3_voltage < 0.1)
  {
    ch3_voltage = 0.00;
  }
  events.send(String(ch3_voltage).c_str(), "ch3voltage", millis()); // update the web page

  reading = ((byte0 << 8 | byte1) >> 2) & 0xFFF; // << THIS IS THE CORRECT ONE! (Thanks Marco!)
}

// Display new GPS info
void displayGPSInfo()
{
  utc_date = ""; // reset concatenated date
  utc_time = ""; // reset concatenated time

  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      utc_time.concat("0");
    utc_time.concat(gps.time.hour());
    utc_time.concat(":");
    if (gps.time.minute() < 10)
      utc_time.concat("0");
    utc_time.concat(gps.time.minute());
    utc_time.concat(":");
    if (gps.time.second() < 10)
      utc_time.concat("0");
    utc_time.concat(gps.time.second());
    Serial.print("Time: ");
    Serial.print(utc_time);
    events.send(String(utc_time).c_str(), "utctime", millis()); // update the web page
    WebSerial.print("GPS Time: ");
    WebSerial.print(utc_time);

    if (gps.date.year() < 10)
      utc_date.concat("0");
    utc_date.concat(gps.date.year());
    utc_date.concat("/");
    if (gps.date.month() < 10)
      utc_date.concat("0");
    utc_date.concat(gps.date.month());
    utc_date.concat("/");
    if (gps.date.day() < 10)
      utc_date.concat("0");
    utc_date.concat(gps.date.day());
    Serial.print(", Date: ");
    Serial.println(utc_date);
    events.send(String(utc_date).c_str(), "utcdate", millis()); // update the web page
    WebSerial.print(", GPS Date: ");
    WebSerial.println(utc_date);

    iso_date = utc_date;        // used for the API
    iso_time = utc_time;        // used for the API
    iso_date.replace("/", "-"); // used for the API
    iso_date.concat("T");       // used for the API
    iso_time.concat(".000Z");   // used for the API
  }
  else
  {
    Serial.println(F("GPS time not yet valid"));
    Serial.println("GPS time not yet valid");
    WebSerial.println(F("GPS time not yet valid"));
    WebSerial.println("GPS time not yet valid");
  }

  if (gps.satellites.isValid())
  {
    Serial.print(F("Sats: "));
    Serial.print(gps.satellites.value());
    events.send(String(gps.satellites.value()).c_str(), "sats", millis()); // update the web page
    WebSerial.print("Sats: ");
    WebSerial.print(gps.satellites.value());
  }

  if (gps.hdop.isValid() && gps.hdop.value() != 0)
  {
    Serial.print(F(", HDOP: "));
    Serial.print(gps.hdop.hdop());                                  // TinyGPS reports DOPs in 100ths
    events.send(String(gps.hdop.hdop()).c_str(), "hdop", millis()); // update the web page
    WebSerial.print(", HDOP: ");
    WebSerial.print(gps.hdop.hdop());
  }
  else
  {
    Serial.print(", HDOP: INVALID");
    WebSerial.print(", HDOP: INVALID");
    events.send(String(gps.hdop.hdop()).c_str(), "hdop", millis()); // update the web page to allow the snackbar to show
  }

  if (gps.location.isValid())
  {
    snprintf(gpsLAT, sizeof gpsLAT, "%lf", gps.location.lat()); // this is some weird workaround to get more than 2 decimals in the string
    snprintf(gpsLON, sizeof gpsLON, "%lf", gps.location.lng());
    Serial.print(", Location: ");
    Serial.print(gpsLAT);
    events.send(gpsLAT, "lat", millis()); // update the web page
    WebSerial.print(", LAT: ");
    WebSerial.print(gpsLAT);
    Serial.print(F(", "));
    Serial.print(gpsLON);
    events.send(gpsLON, "lon", millis()); // update the web page
    WebSerial.print(", LON: ");
    WebSerial.print(gpsLON);
    events.send(String(gps.location.age() / 1000).c_str(), "gpsages", millis()); // update the web page
  }
  else
  {
    Serial.print(", LAT: INVALID");
    Serial.print(", LON: INVALID");
    WebSerial.print(", LAT: INVALID");
    WebSerial.print(", LON: INVALID");
  }

  if (gps.altitude.isValid())
  {
    Serial.print(F(", Altitude Meters: "));
    Serial.println(gps.altitude.meters());
    events.send(String(gps.altitude.meters()).c_str(), "alt", millis()); // update the web page
    WebSerial.print(", ALT: ");
    WebSerial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println(", ALT: INVALID"); // Done printing alt, siv, hdop
    WebSerial.println(", ALT: INVALID");
  }
}

void getGPSData()
{
  while (myI2CGPS.available())
  {                              // available() returns the number of new bytes available from the GPS module
    gps.encode(myI2CGPS.read()); // Feed the GPS parser
  }
  if (gps.time.isUpdated())
  { // Check to see if new GPS info is available
    displayGPSInfo();
  }
}

void printSensorData()
{
  Serial.println((String) "ADC: Board temp: " + ch0_temperature + ", Vin: " + ch0_voltage + ", CH1 V: " + ch1_voltage + ", CH2 V: " + ch2_voltage + ", CH3 V: " + ch3_voltage);
  WebSerial.println((String) "ADC: Board temp: " + ch0_temperature + ", Vin: " + ch0_voltage + ", CH1 V: " + ch1_voltage + ", CH2 V: " + ch2_voltage + ", CH3 V: " + ch3_voltage);
}

void manageWarnings()
{
  if ((gps.hdop.hdop() > minHDOPAllowed) || (gps.hdop.hdop() == 0))
  {
    warnings2 = "GPS HDOP high!";
    warnings0 = "";
    events.send(warnings2.c_str(), "warnings2", millis()); // update the web page
    events.send(warnings0.c_str(), "warnings0", millis()); // update the web page
  }
  else
  {
    warnings2 = "";
    events.send(warnings2.c_str(), "warnings2", millis()); // update the web page
  }
  if (!gps.location.isValid())
  {
    warnings1 = "GPS location data invalid!";
    warnings0 = "";
    events.send(warnings1.c_str(), "warnings1", millis()); // update the web page
    events.send(warnings0.c_str(), "warnings0", millis()); // update the web page
  }
  else
  {
    warnings1 = "";
    events.send(warnings1.c_str(), "warnings1", millis()); // update the web page
  }
  if (ch0_temperature > 80.00)
  {
    warnings3 = "Enclosure temp extreme!";
    warnings0 = "";
    events.send(warnings3.c_str(), "warnings3", millis()); // update the web page
    events.send(warnings0.c_str(), "warnings0", millis()); // update the web page
  }
  else if (ch0_temperature > 75.00)
  {
    warnings3 = "Enclosure temp high!";
    warnings0 = "";
    events.send(warnings3.c_str(), "warnings3", millis()); // update the web page
    events.send(warnings0.c_str(), "warnings0", millis()); // update the web page
  }
  else if (ch0_temperature > 55.00)
  {
    warnings3 = "Enclosure fan running!";
    warnings0 = "";
    events.send(warnings3.c_str(), "warnings3", millis()); // update the web page
    events.send(warnings0.c_str(), "warnings0", millis()); // update the web page
  }
  else
  {
    warnings3 = "";
    events.send(warnings3.c_str(), "warnings3", millis()); // update the web page
  }
  if (warnings1 == "" && warnings2 == "" && warnings3 == "")
  {
    warnings0 = " None!";
    events.send(warnings3.c_str(), "warnings0", millis()); // update the web page
    events.send(warnings0.c_str(), "warnings0", millis()); // update the web page
    WebSerial.println("Warnings: None!");
    Serial.println("Warnings: None!");
  }
  else
  {
    WebSerial.print("Warnings: ");
    Serial.print("Warnings: ");
    if (warnings1 != "")
    {
      WebSerial.print(warnings1);
      Serial.print(warnings1);
    }
    if (warnings1 != "" && warnings2 != "")
    {
      WebSerial.print(", ");
      Serial.print(", ");
    }
    if (warnings2 != "")
    {
      WebSerial.print(warnings2);
      Serial.print(warnings2);
    }
    if (warnings2 != "" && warnings3 != "")
    {
      WebSerial.print(", ");
      Serial.print(", ");
    }
    if (warnings3 != "")
    {
      WebSerial.print(warnings3);
      Serial.print(warnings3);
    }
    WebSerial.println();
    Serial.println();
  }
}

// run through the functions that gather data
void getSensorData()
{
  manageWarnings();     // generate any warnings from sensor or environmental data
  calculateUptime();    // calulates the AE_CM's uptime as xd:xh:xm:xs
  getGPSData();         // GPS location, stats and time
  readFromADC();        // Temperature and voltages
  printSensorData();    // call the function that prints out sensor data
  getFANState();        // get the FAN's power state
  getGPSAntennaState(); // get the FAN's power state
  // sendUDP();                        // TODO: finish the UDP sender and enable it
  Serial.println();    // put a new line at the end of the Serial output
  WebSerial.println(); // put a new line at the end of the WebSerial output
}

void parseBytes(const char *str, char sep, byte *bytes, int maxBytes, int base)
{
  for (int i = 0; i < maxBytes; i++)
  {
    bytes[i] = strtoul(str, NULL, base); // Convert byte
    str = strchr(str, sep);              // Find next separator
    if (str == NULL || *str == '\0')
    {
      break; // No more separators, exit
    }
    str++; // Point to next character after separator
  }
}

void softFactoryReset()
{
  hostname = "AE-CM";                                        // stores the hostname
  String friendly_product = (String(PRODUCT).c_str());       // stores compile-time date
  String friendly_version = (String(BOARD_VERSION).c_str()); // stores compile-time time
  friendly_name = friendly_product + " " + friendly_version; // stores friendly name
  static_ip_en = prod_release;                               // stores whether a static IP is enabled
  ipaddressL = "192.168.137.137";                            // stores static IPv4 address
  subnetL = "255.255.255.0";                                 // stores static IPv4 subnet
  gatewayL = "192.168.137.1";                                // stores static IPv4 gateway
  dnsL = "8.8.8.8";                                          // stores static IPv4 dns server address
  ch0_name = "Caravan Battery";                              // stores ADC CH 0 label
  ch1_name = "Caravan Solar";                                // stores ADC CH 1 label
  ch2_name = "SPARE";                                        // stores ADC CH 2 label
  ch3_name = "SPARE";                                        // stores ADC CH 3 label
  relay1_name = "UNUSED";                                    // stores relay1 label
  relay2_name = "UNUSED";                                    // stores relay2 label
  http_username = "admin";                                   // stores username for the webage
  http_password = "admin";                                   // stores password for the webage
  ap_enabled = 0;                                            // stores username for the webage
  ap_ssid = "AE-CONFIGURE";                                  // stores AP network SSID
  ap_password = "password";                                  // stores AP network password
  ap_ip_address = "192.168.4.1";                             // stores AP network IP address
  ap_subnet = "255.255.255.0";                               // stores AP network subnet
  ap_gateway = "192.168.4.1";                                // stores AP network gateway
  sta_enabled = 0;                                           // stores username for the webage
  sta_ssid = "your network";                                 // stores STA network SSID
  sta_password = "your password";                            // stores STA network password
  sta_ip_address = "192.168.0.250";                          // stores STA network IP address
  sta_subnet = "255.255.255.0";                              // stores STA network subnet
  sta_gateway = "192.168.0.1";                               // stores STA network gateway
  sta_dnsL = "8.8.8.8";                                      // stores STA DNS
  udp_payload_en = 0;                                        // stores UDP sender state
  udp_payload_ip = "192.168.137.1";                          // stores UDP sender IP
  udp_payload_p = 2331;                                      // stores UDP sender port
  eth_enabled = 0;
}

void launchAP()
{
  IPAddress CheckIP = WiFi.softAPIP();
  IPAddress badIP = {0,0,0,0};
  Serial.print("CheckIP is: ");
  Serial.println(CheckIP);
  
  if (CheckIP != badIP)
  {
    Serial.println("SoftAP already running, returning...");
    return;
  }
  else
  {
    Serial.println("SoftAP is being configured...");
  }

  // Configures static IP address
  IPAddress apip, apsn, apgw, apns; // create some class objects to use below

  apip.fromString(ap_ip_address); // converts the ipaddressL (ipaddress ip) string into a 4 byte array for you with the IPAddress class
  apgw.fromString(ap_gateway);    // converts the gatewayL (gateway ip) string into a 4 byte array for you with the IPAddress class
  apsn.fromString(ap_subnet);     // converts the subnetL (subnet ip) string into a 4 byte array for you with the IPAddress class

  if (!WiFi.softAPConfig(apip, apgw, apsn))
  {
    Serial.println("Wifi AP failed to configure, crashing loop");
  }
  else
  {
    Serial.println("AE-CM created AE-CONFIGURE WiFi AP network!");
    WiFi.mode(WIFI_MODE_AP);
    Serial.print("AP enabed with SSID: ");
    Serial.println(ap_ssid.c_str());
    Serial.print("AP enabed with Password: ");
    Serial.println(ap_password.c_str());
    WiFi.softAP("AE-CONFIGURE", "password");
    Serial.print("AP enabed! AE-CM IP: ");
    Serial.println(WiFi.softAPIP());
  }
}

void launchSTA() 
{
  WiFi.mode(WIFI_STA); 
  WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
  Serial.print("Connecting to WiFi ..");
  delay(2000);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("AE-CM WiFi connection unsuccessful...");
  }
  Serial.println(WiFi.localIP());
  Serial.println("AE-CM connected to WiFi network!");
}

void flashSYSLED(int times)
{
  for (int loops = 1; loops < times + 1; loops++)
  {
    digitalWrite(SYS_LED, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(250);                  // wait for a second
    digitalWrite(SYS_LED, LOW);  // turn the LED off by making the voltage LOW
    delay(250);
  }
}

void sendUDP()
{ // TODO: finish and use this
  char *my_data_to_send = (char *)"Hello!";
  Udp.beginPacket(remoteIP, udp_payload_p); // Ready to send data
  for (int i = 0; i < strlen(my_data_to_send); i++)
  {
    Udp.write((uint8_t)my_data_to_send[i]);
  }
  if (!Udp.endPacket())
  {
    Serial.println("UDP data has not been sent");
    WebSerial.println("UDP data has not been sent");
  }
  else
  {
    Serial.println("UDP data has been sent");
    WebSerial.println("UDP data has been sent");
  }
  // Udp.write((const uint8_t*)buf, packetSize); // Copy data to send buffer
  Udp.endPacket();
}

void erasePreferences()
{
  preferences.begin("settings", false);
  preferences.clear();
  preferences.end();
  Serial.println("Preferences Erased...");
}

// runs once at boot
void setup()
{
  esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               // add current thread to WDT watch

  Serial.begin(115200); // enable serial output

  mrd = new MultiResetDetector(MRD_TIMEOUT, MRD_ADDRESS); // initialise the multi-reset detector

  if (mrd->detectMultiReset())
  {
    delay(3000);
    Serial.println("Multi Reset Detected");
    MRD_Detected = true;
    erasePreferences();
    flashSYSLED(2);     // blink the SYS LED a couple of times for confirmation
    saveOnly = true;    // stops syncSettings from reading before writing, esentially overwriting what was stored
    softFactoryReset(); // set all strings to default values
    syncSettings();     // synces all strings with persistent storage and performs a factory reset, because "saveOnly = true;"
    saveOnly = false;   // put her back to stock for when she goes over the pits
    ap_enabled = true;   // allows the AP to fire up
    launchAP(); // this is a function so that it can also be called from the MRD
  }
  else
  {
    delay(3000);
    Serial.println("No Multi Reset Detected, continuing boot...");
    BootReason = esp_reset_reason();
    if (BootReason == 1)
    { // Reset due to power-on event.
      Serial.println("Reboot was because of Power-On!!");
    }

    if (BootReason == 6)
    { // Reset due to task watchdog.
      Serial.println("Reboot was because of WDT!!");
    }

    softFactoryReset(); // resets all the strings, an actual factory reset only happens when a multi reset occurs

    syncSettings(); // sync all the strings from persitent storage and overwrites what softFactoryReset wrote, essintially loading the last saved state(config)

    if ((ap_enabled) || ((!ap_enabled) && (!sta_enabled) && (!eth_enabled))) // ToDO: add some error checking, make sure there is a configured SSID ad PWD
    {
      ap_enabled = true;
      Serial.println("Wifi AP starting...");
      launchAP(); // this is a function so that it can also be called from the MRD
    }
    else if (sta_enabled) // ToDO: add some error checking, make sure there is a configured SSID ad PWD
    {
      launchSTA();
    }
    else if (eth_enabled)
    {
      WiFi.onEvent(WiFiEvent); // initialise all the parts from the WiFi library that Ethernet is going to need
      ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // start Ethernet // ToDo: move this into the loop and disable if not needed

      ETH.setHostname(hostname.c_str()); // take the string "hostname" and make it a string buffer
      Serial.print("Hostname: ");
      Serial.println(ETH.getHostname());

      if (static_ip_en == "True" && ipaddressL != "" && ipaddressL != "0.0.0.0" && ipaddressL != "1.0.0.0" && subnetL != "" && gatewayL != "")
      {
        IPAddress ip, sn, gw, ns; // create some class objects to use below

        ip.fromString(ipaddressL); // converts the ipaddressL (ipaddress ip) string into a 4 byte array for you with the IPAddress class
        sn.fromString(subnetL);    // converts the subnetL (subnet ip) string into a 4 byte array for you with the IPAddress class
        gw.fromString(gatewayL);   // converts the gatewayL (gateway ip) string into a 4 byte array for you with the IPAddress class
        if (dnsL == "")
        {
          dnsL = "0.0.0.0";
        }
        else
        {
          ns.fromString(dnsL); // converts the dnsL (dns ip) string into a 4 byte array for you with the IPAddress class
        }

        Serial.println("Static IP enabled in settings, pulling data from flash and reconfiguring interface!");
        if (dnsL == "")
        {
          ETH.config(ip, gw, sn); // this is where we configure the Ethernet interface with the static info, if enabled
        }
        else
        {
          ETH.config(ip, gw, sn, ns); // this is where we configure the Ethernet interface with the static info, if enabled
        }
      }
      else
      {
        Serial.println("DHCP enabled in settings or static settings are corrupt, enabling DHCP and requesting an IP!");
        delay(3000);       // need some time for the IP info to negotiate
        if (eth_connected) // only wait for DHCP if Ethernet is connected
        {
          while (!((uint32_t)ETH.localIP()) && WiFi.status() != WL_CONNECTED)
          {
            // wait here for a DHCP IP on Ethernet, forever. No IP then no AE-CM
            Serial.println("DHCP is enabled, but, we haven't yet been given an address...");
            delay(3000);
          }
          Serial.print("Ethernet IP address: ");
          Serial.println(ETH.localIP());
        }
      }
    }
    else
    {
      stopWIFI();
    }
  }

  pinMode(SYS_LED, OUTPUT); // enable GPIO as output for SYS LED

  delay(1000); // give seriala a bit to initilise

  Wire.begin(33, 32); // enable I2C by Wire

  WiFi.setHostname(hostname.c_str()); // this is needed to actually set the hostname

  getSensorDataTimer.setInterval(10000L, getSensorData);    // get sensor data every 10 seconds
  updateMDNSDataTimer.setInterval(600000L, updateMDNSData); // update MDNS data every 10 minutes

  vspi = new SPIClass(VSPI); // initialise vSPI

  pinMode(FAN_ON, INPUT);               // set the GPIO attached to the FAN power
  pinMode(CS, OUTPUT);                  // set the GPIO attached to the ADC SS/CS pin (SS workaround)
  pinMode(EXTERNAL_GPS_ANTENNA, INPUT); // set the GPIO attached to the CH0 ADC switch to an OUTPUT
  pinMode(TEMP_SENSOR_ON, OUTPUT);      // set the GPIO attached to the CH0 ADC switch to an OUTPUT
  pinMode(RELAY_1_ON, OUTPUT);          // set the GPIO attached to Relay1 to an OUTPUT
  pinMode(RELAY_2_ON, OUTPUT);          // set the GPIO attached to Relay2 to an OUTPUT

  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { // Route for root / web page
    request->send(LittleFS, "/index.html", String(), false, processor);
  });

  // ToDo: add warnings to the api?
  server.on("/api", HTTP_GET, [](AsyncWebServerRequest *request) { // Route for root / web page
    api_string = (String) "{ \"ethip\": \"" + ETH.localIP().toString() + "\", \"ethsubnet\": \"" + ETH.subnetMask().toString() + "\", "
                                                                                                                                 "\"ethgateway\": \"" +
                 ETH.gatewayIP().toString() + "\", \"ethdns\": \"" + ETH.dnsIP().toString() + "\", "
                                                                                              "\"wifiip\": \"" +
                 WiFi.localIP().toString() + "\", \"wifisubnet\": \"" + WiFi.subnetMask().toString() + "\", "
                                                                                                       "\"wifigateway\": \"" +
                 WiFi.gatewayIP().toString() + "\", \"wifidns\": \"" + WiFi.dnsIP().toString() + "\", "
                                                                                                 "\"latitude\": " +
                 gpsLAT + ", \"longitude\": " + gpsLON + ", "
                                                         "\"altitudem\": " +
                 gps.altitude.meters() + ", \"sats\": " + gps.satellites.value() + ", "
                                                                                   "\"hdop\": " +
                 gps.hdop.hdop() + ", \"gpsages\": " + gps.location.age() / 1000 + ", "
                                                                                   "\"freeheapb\": " +
                 ESP.getFreeHeap() + ", \"hostname\": \"" + hostname + "\", "
                                                                       "\"enclosuretempc\": " +
                 ch0_temperature + ", \"enclosurefan\": \"" + enclosure_fan + "\", "
                                                                              "\"gpsantenna\": \"" +
                 gps_antenna + "\", \"timestamp\": \"" + iso_date + iso_time + "\", "
                                                                               "\"vinvoltage\": " +
                 ch0_voltage + ", \"ch1voltage\": " + ch1_voltage + ", "
                                                                    "\"ch2voltage\": " +
                 ch2_voltage + ", \"ch3voltage\": " + ch3_voltage + ", "
                                                                    "\"ch0name\": \"" +
                 ch0_name + "\", \"ch1name\": \"" + ch1_name + "\", "
                                                               "\"ch2name\": \"" +
                 ch2_name + "\", \"ch3name\": \"" + ch3_name + "\", "
                                                               "\"relay1state\": \"" +
                 relay1_state + "\", \"relay2state\": \"" + relay2_state + "\", "
                                                                           "\"relay1name\": \"" +
                 relay1_name + "\", \"relay2name\": \"" + relay2_name + "\", "
                                                                        "\"ethmacaddress\": \"" +
                 ETH.macAddress() + "\", \"cputempc\": " + ((temprature_sens_read() - 32) / 1.8) + ", "
                                                                                                   "\"devicelabel\": \"" +
                 friendly_name + "\", \"hwversion\": \"" + ae_cm_version + "\", "
                                                                           "\"fwversion\": \"" +
                 firmware_version + "\", \"wifimacaddress\": \"" + WiFi.macAddress() + "\", "
                                                                                       "\"staticethipen\": \"" +
                 static_ip_en + "\", \"uptime\": \"" + uptimeString + "\" "
                                                                      "}";
    // ToDo: add the WiFi info
    request->send(200, "text/plain", api_string);
  });

  // this is here for backwards compatibility
  server.on("/location_data", HTTP_GET, [](AsyncWebServerRequest *request) { // Route for root / web page
    api_string = (String) "{ \"latitude\": " + gpsLAT + ", \"longitude\": " + gpsLON + ", "
                                                                                       "\"altitude\": " +
                 gps.altitude.meters() + ", \"sats\": " + gps.satellites.value() + ", "
                                                                                   "\"hdop\": " +
                 gps.hdop.hdop() + ", \"gpsages\": " + gps.location.age() / 1000 + ", "
                                                                                   "\"gpsantenna\": \"" +
                 gps_antenna + "\", \"timestamp\": \"" + iso_date + iso_time + "\" "
                                                                               "}";
    request->send(200, "text/plain", api_string);
  });

  // this is here for backwards compatibility
  server.on("/sensor_data", HTTP_GET, [](AsyncWebServerRequest *request) { // Route for root / web page
    api_string = (String) "{ \"batt_voltage\": " + ch0_voltage + ", \"batt_current\": " + ch3_voltage + ", "
                                                                                                        "\"solar_current\": " +
                 ch2_voltage + ", \"solar_voltage\": " + ch1_voltage + " "
                                                                       "}";
    request->send(200, "text/plain", api_string);
  });

  server.on("/io.html", HTTP_GET, [](AsyncWebServerRequest *request) { // Route for settings web page
    if (!request->authenticate(http_username.c_str(), http_password.c_str()))
      return request->requestAuthentication();
    request->send(LittleFS, "/io.html", String(), false, processor);
  });

  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *request) { // Route for settings web page
    if (!request->authenticate(http_username.c_str(), http_password.c_str()))
      return request->requestAuthentication();
    request->send(LittleFS, "/settings.html", String(), false, processor);
  });

  // Send a GET request to <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
  server.on("/savedata", HTTP_GET, [](AsyncWebServerRequest *request) {
      int args = request->args();
      for(int i=0;i<args;i++){
          Serial.printf("ARG[%s]: %s\n", request->argName(i).c_str(), request->arg(i).c_str());
          WebSerial.print("Settings saved via web interface!");
      }
      preferences.begin("settings", false); //create an array named 'settings' to store setting data
      bool restartNeeded = false;
      String inputMessage1,inputMessage2,inputMessage3,inputMessage4,inputMessage5,inputMessage6, \
              inputMessage7,inputMessage8,inputMessage9,inputMessage10,inputMessage11,inputMessage12, \
              inputMessage13,inputMessage14,inputMessage15,inputMessage16,inputMessage17,inputMessage18, \
              inputMessage19,inputMessage20,inputMessage21,inputMessage22,inputMessage23,inputMessage24, \
              inputMessage25,inputMessage26,inputMessage27,inputMessage28,inputMessage29,inputMessage30, \
              inputMessage31;
              
      if (request->hasParam(input_parameter1)) {
          //hostname
          if ((request->getParam(input_parameter1)->value().length() > 0) && (request->getParam(input_parameter1)->value() != hostname)) {
              inputMessage1 = request->getParam(input_parameter1)->value(); // assing the URLParam value
              hostname = inputMessage1;
              if (hostname.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected in hostname, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected in hostname, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("hostname", (String(hostname).c_str())); // add new setting to flash storage
              settings_saved = true;
              restartNeeded = true;
              events.send(String(hostname).c_str(),"hostname",millis()); // update the web page
              Serial.println("hostname updated via web settings");
              WebSerial.println("hostname updated via web settings");
          }
      } 
      if (request->hasParam(input_parameter2)) {
          //friendly_name
          if ((request->getParam(input_parameter2)->value().length() > 0) && (request->getParam(input_parameter2)->value() != friendly_name)) {
              inputMessage2 = request->getParam(input_parameter2)->value(); // assing the URLParam value
              friendly_name = inputMessage2;
              if (friendly_name.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected in devicename, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected in devicename, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("friendly_name", (String(friendly_name).c_str())); // add new setting to flash storage
              settings_saved = true;
              restartNeeded = true;
              Serial.println("friendly_name updated via web settings");
              WebSerial.println("friendly_name updated via web settings");
          }
      }
      if (request->hasParam(input_parameter3)) {
          //ip_address
          if ((request->getParam(input_parameter3)->value().length() > 0) && (request->getParam(input_parameter3)->value() != ipaddressL)) {
              inputMessage3 = request->getParam(input_parameter3)->value(); // assing the URLParam value
              ipaddressL = inputMessage3;
              if (ipaddressL.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected in IP address, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected in IP address, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              if (String(inputMessage3) != ipaddressL) {
                  Serial.println("IP address settings updated");
                  WebSerial.println("IP address settings updated");
              }
              preferences.putString("ip_address", (String(ipaddressL).c_str())); // add new setting to flash storage
              settings_saved = true;
              restartNeeded = true;
              Serial.println("ip_address updated via web settings");
              WebSerial.println("ip_address updated via web settings");
          }
      }
      if (request->hasParam(input_parameter4)) {
          //subnet
          if ((request->getParam(input_parameter4)->value().length() > 0) && (request->getParam(input_parameter4)->value() != subnetL)) {
              inputMessage4 = request->getParam(input_parameter4)->value(); // assing the URLParam value
              subnetL = inputMessage4;
              if (subnetL.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected in subnet, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected in subnet, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              if (String(inputMessage4) != subnetL) {
                  Serial.println("Subnet settings updated, restart scheduled");
                  WebSerial.println("Subnet settings updated, restart scheduled");
              }
              preferences.putString("subnet", (String(subnetL).c_str())); // add new setting to flash storage
              settings_saved = true;
              restartNeeded = true;
              Serial.println("subnet updated via web settings");
              WebSerial.println("subnet updated via web settings");
          }
      }
      if (request->hasParam(input_parameter5)) {
          //gateway
          if ((request->getParam(input_parameter5)->value().length() > 0) && (request->getParam(input_parameter5)->value() != gatewayL)) {
              inputMessage5 = request->getParam(input_parameter5)->value(); // assing the URLParam value
              gatewayL = inputMessage5;
              if (gatewayL.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected in gateway, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected in gateway, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              if (String(inputMessage5) != gatewayL) {
                  Serial.println("Gateway settings updated");
                  WebSerial.println("Gateway settings updated");
              }
              preferences.putString("gateway", (String(gatewayL).c_str())); // add new setting to flash storage
              settings_saved = true;
              restartNeeded = true;
              Serial.println("gateway updated via web settings");
              WebSerial.println("gateway updated via web settings");
          }
      } 
      if (request->hasParam(input_parameter5)) {
          //dns
          if (request->getParam(input_parameter6)->value() != dnsL) {
              inputMessage6 = request->getParam(input_parameter6)->value(); // assing the URLParam value
              if (String(input_parameter6) != dnsL) {
                  Serial.println("DNS settings updated");
                  WebSerial.println("DNS settings updated");
              }
              dnsL = inputMessage6;
              if (dnsL.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("dns", (String(dnsL).c_str())); // add new setting to flash storage
              settings_saved = true;
              restartNeeded = true;
              Serial.println("dns updated via web settings");
              WebSerial.println("dns updated via web settings");
          }
      }
      if (request->hasParam(input_parameter7)) {
          //ch1name
          if ((request->getParam(input_parameter7)->value().length() > 0) && (request->getParam(input_parameter7)->value() != ch1_name)) {
              inputMessage7 = request->getParam(input_parameter7)->value(); // assing the URLParam value
              ch1_name = inputMessage7;
              if (ch1_name.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("ch1_name", (String(ch1_name).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(ch1_name).c_str(),"ch1name",millis()); // update the web page
              Serial.println("ch1name updated via web settings");
              WebSerial.println("ch1name updated via web settings");
          }
      }
      if (request->hasParam(input_parameter8)) {
          //ch2name
          if ((request->getParam(input_parameter8)->value().length() > 0) && (request->getParam(input_parameter8)->value() != ch2_name)) {
              inputMessage8 = request->getParam(input_parameter8)->value();
              ch2_name = inputMessage8;
              if (ch2_name.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("ch2_name", (String(ch2_name).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(ch2_name).c_str(),"ch2name",millis()); // update the web page
              Serial.println("ch2name updated via web settings");
              WebSerial.println("ch2name updated via web settings");
          }
      }
      if (request->hasParam(input_parameter9)) {
          //ch3name
          if ((request->getParam(input_parameter9)->value().length() > 0) && (request->getParam(input_parameter9)->value() != ch3_name)) {
              inputMessage9 = request->getParam(input_parameter9)->value(); // assing the URLParam value
              ch3_name = inputMessage9;
              if (ch3_name.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("ch3_name", (String(ch3_name).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(ch3_name).c_str(),"ch3name",millis()); // update the web page
              Serial.println("ch3name updated via web settings");
              WebSerial.println("ch3name updated via web settings");
          }
      } 
      if (request->hasParam(input_parameter10)) {
          //relay1_name
          if ((request->getParam(input_parameter10)->value().length() > 0) && (request->getParam(input_parameter10)->value() != relay1_name)) {
              inputMessage10 = request->getParam(input_parameter10)->value(); // assing the URLParam value
              relay1_name = inputMessage10;
              if (relay1_name.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("relay1_name", (String(relay1_name).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(relay1_name).c_str(),"relay1name",millis()); // update the web page
              Serial.println("relay1_name updated via web settings");
              WebSerial.println("relay1_name updated via web settings");
          }
      }
      if (request->hasParam(input_parameter11)) {
          //relay2name
          if ((request->getParam(input_parameter11)->value().length() > 0) && (request->getParam(input_parameter11)->value() != relay2_name)) {
              inputMessage11 = request->getParam(input_parameter11)->value(); // assing the URLParam value
              relay2_name = inputMessage11;
              if (relay2_name.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("relay2_name", (String(relay2_name).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(relay2_name).c_str(),"relay2name",millis()); // update the web page
              Serial.println("relay2name updated via web settings");
              WebSerial.println("relay2name updated via web settings");
          }
      }
      if (request->hasParam(input_parameter12)) {
          //static_ip_en (is static IP enabled)
          if (static_ip_en == "False") {
              static_ip_en = "True"; 
              preferences.putString("static_ip_en", (String(static_ip_en).c_str())); // add new setting to flash storage
              events.send(String(static_ip_en).c_str(),"static_ip_en",millis()); // update the web page
              restartNeeded = true;
              Serial.println("Static IP enabled in settings, restart scheduled");
              WebSerial.println("Static IP enabled in settings, restart scheduled");
          } else {
              restartNeeded = false;
              Serial.println("Static IP mode still enabled");
              WebSerial.println("Static IP mode still enabled");
          }
      } else if (!request->hasParam(input_parameter12)) {
          static_ip_en = "False"; 
          preferences.putString("static_ip_en", (String(static_ip_en).c_str())); // add new setting to flash storage
          settings_saved = true;
          events.send(String(static_ip_en).c_str(),"static_ip_en",millis()); // update the web page
          restartNeeded = true;
          Serial.println("DHCP enabled in settings, restart scheduled");
          WebSerial.println("DHCP enabled in settings, restart scheduled");
      }
      if (request->hasParam(input_parameter13)) {
          //http_username
          if ((request->getParam(input_parameter13)->value().length() > 0) && (request->getParam(input_parameter13)->value() != http_username)) {
              inputMessage13 = request->getParam(input_parameter13)->value(); // assing the URLParam value
              http_username = inputMessage13;
              if (http_username.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("http_username", (String(http_username).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(http_username).c_str(),"http_username",millis()); // update the web page
              Serial.println("http_username updated via web settings");
              WebSerial.println("http_username updated via web settings");
          }
      }
      if (request->hasParam(input_parameter14)) {
          //http_password
          if ((request->getParam(input_parameter14)->value().length() > 0) && (request->getParam(input_parameter14)->value() != http_password)) {
              inputMessage14 = request->getParam(input_parameter14)->value(); // assing the URLParam value
              http_password = inputMessage14;
              if (http_password.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("http_password", (http_password)); // add new setting to flash storage
              settings_saved = true;
              events.send(http_password.c_str(),"http_password",millis()); // update the web page
              Serial.println("http_password updated via web settings");
              WebSerial.println("http_password updated via web settings");
          }
      }
      if (request->hasParam(input_parameter15)) {
            //sta_ssid
            if ((request->getParam(input_parameter15)->value().length() > 0) && (request->getParam(input_parameter15)->value() != sta_ssid)) {
                inputMessage15 = request->getParam(input_parameter15)->value(); // assing the URLParam value
                sta_ssid = inputMessage15.c_str();
                if (sta_ssid.indexOf(";") != -1) {
                    WebSerial.println(sta_ssid.indexOf(";"));
                    WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                    Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                    return; // crash the loop and don't save the 1337 h4x0r script
                }
                preferences.putString("sta_ssid", sta_ssid); // add new setting to flash storage
                settings_saved = true;
                events.send(sta_ssid.c_str(),"sta_ssid",millis()); // update the web page
                Serial.println("sta_ssid updated via web settings");
                WebSerial.println("sta_ssid updated via web settings");
            }
        }
        if (request->hasParam(input_parameter16)) {
            //sta_password
            if ((request->getParam(input_parameter16)->value().length() > 0) && (request->getParam(input_parameter16)->value() != sta_password)) {
                inputMessage16 = request->getParam(input_parameter16)->value(); // assing the URLParam value
                sta_password = inputMessage16.c_str();
                if (sta_password.indexOf(";") != -1) {
                    WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                    Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                    return; // crash the loop and don't save the 1337 h4x0r script
                }
                preferences.putString("sta_password", sta_password); // add new setting to flash storage
                settings_saved = true;
                events.send(sta_password.c_str(),"stapwd",millis()); // update the web page
                Serial.println("sta_password updated via web settings");
                WebSerial.println("sta_password updated via web settings");
            }
        }
        if (request->hasParam(input_parameter17) && (request->hasParam("stasourcecheck"))) {
            //sta_enabled
            if (sta_enabled != 1) {
                sta_enabled = 1;
                restartNeeded = true;
                preferences.putBool("sta_enabled", 1); // add new setting to flash storage
                settings_saved = true;
                events.send("1","staenabled",millis()); // update the web page
                Serial.println("Station mode enabled, restart scheduled");
                WebSerial.println("Station mode enabled, restart scheduled");
            } else {
                restartNeeded = false;
                Serial.println("Station mode is still enabled");
                WebSerial.println("Station mode is still enabled");
            }
        } else if (request->hasParam("stasourcecheck")){
            sta_enabled = 0;
            restartNeeded = true;
            preferences.putBool("sta_enabled", 0); // add new setting to flash storage
            settings_saved = true;
            events.send("0","staenabled",millis()); // update the web page
            Serial.println("Station mode disabled");
            WebSerial.println("Station mode disabled");
        }
      if (request->hasParam(input_parameter18)) {
          //ap_ssid
          if ((request->getParam(input_parameter18)->value().length() > 0) && (request->getParam(input_parameter18)->value() != ap_ssid)) {
              inputMessage18 = request->getParam(input_parameter18)->value(); // assing the URLParam value
              ap_ssid = inputMessage18.c_str();
              if (ap_ssid.indexOf(";") != -1) {
                  WebSerial.println("a Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("ap_ssid", ap_ssid); // add new setting to flash storage
              settings_saved = true;
              events.send(ap_ssid.c_str(),"apssid",millis()); // update the web page
              Serial.println("ap_ssid updated via web settings");
              WebSerial.println("ap_ssid updated via web settings");
          }
      }
      if (request->hasParam(input_parameter19)) {
          //ap_password
          if ((request->getParam(input_parameter19)->value().length() > 0) && (request->getParam(input_parameter19)->value() != ap_password)) {
              inputMessage19 = request->getParam(input_parameter19)->value(); // assing the URLParam value
              ap_password = inputMessage19.c_str();
              if (ap_password.indexOf(";") != -1) {
                  WebSerial.println("b Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("ap_password", ap_password); // add new setting to flash storage
              settings_saved = true;
              events.send(ap_password.c_str(),"appwd",millis()); // update the web page
              Serial.println("ap_password updated via web settings");
              WebSerial.println("ap_password updated via web settings");
          }
      }
      if (request->hasParam(input_parameter20) && (request->hasParam("apsourcecheck"))) {
          //ap_enabled
          if (ap_enabled != 1) {
              ap_enabled = 1;
              restartNeeded = true;
              preferences.putBool("ap_enabled", 1); // add new setting to flash storage
              settings_saved = true;
              events.send("1","apenabled",millis()); // update the web page
              Serial.println("AP mode enabled");
              WebSerial.println("AP mode enabled");
          } else {
              restartNeeded = false;
              Serial.println("AP mode still enabled");
              WebSerial.println("AP mode still enabled");
          }
      } else if (request->hasParam("apsourcecheck")){
          ap_enabled = 0;
          restartNeeded = true;
          preferences.putBool("ap_enabled", 0); // add new setting to flash storage
          settings_saved = true;
          events.send("0","apenabled",millis()); // update the web page
          Serial.println("AP mode disabled");
          WebSerial.println("AP mode disabled");
      }
      if (request->hasParam(input_parameter21)) {
          //udp_payload_ip
          if ((request->getParam(input_parameter21)->value().length() > 0) && (request->getParam(input_parameter21)->value() != udp_payload_ip)) {
              inputMessage21 = request->getParam(input_parameter21)->value(); // assing the URLParam value
              if (String(inputMessage21) != udp_payload_ip) {
                  restartNeeded = true;
                  Serial.println("udp_payload_ip settings updated");
                  WebSerial.println("udp_payload_ip settings updated");
              }
              udp_payload_ip = inputMessage21;
              if (udp_payload_ip.indexOf(";") != -1) {
                  WebSerial.println("c Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("udp_payload_p", (String(udp_payload_p).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(udp_payload_ip).c_str(),"udppayloadip",millis()); // update the web page
              Serial.println("udp_payload_ip updated via web settings");
              WebSerial.println("udp_payload_ip updated via web settings");
          }
      } 
      if (request->hasParam(input_parameter22)) {
          //udp_payload_p
          if ((request->getParam(input_parameter22)->value().length() > 0) && (request->getParam(input_parameter22)->value() != (String(udp_payload_p).c_str()))) {
              inputMessage22 = request->getParam(input_parameter22)->value(); // assing the URLParam value
              if (inputMessage22.toInt() != udp_payload_p) {
                  restartNeeded = true;
                  Serial.println("udp_payload_p settings updated");
                  WebSerial.println("udp_payload_p settings updated");
              }
              udp_payload_p = inputMessage22.toInt();
              preferences.putInt("udp_payload_p", udp_payload_p); // add new setting to flash storage
              settings_saved = true;
              events.send(String(udp_payload_p).c_str(),"udppayloadp",millis()); // update the web page
              Serial.println("udp_payload_p updated via web settings");
              WebSerial.println("udp_payload_p updated via web settings");
          }
      } 
      if (request->hasParam(input_parameter23)) {
          //ap_ip_address
          if ((request->getParam(input_parameter23)->value().length() > 0) && (request->getParam(input_parameter23)->value() != ap_ip_address)) {
              inputMessage23 = request->getParam(input_parameter23)->value(); // assing the URLParam value
              ap_ip_address = inputMessage23;
              if (ap_ip_address.indexOf(";") != -1) {
                  WebSerial.println("e Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              if (String(inputMessage23) != ap_ip_address) {
                  restartNeeded = true;
                  Serial.println("ap_ip_address settings updated");
              }
              preferences.putString("ap_ip_address", (String(ap_ip_address).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(ap_ip_address).c_str(),"apipaddress",millis()); // update the web page
              Serial.println("ap_ip_address updated via web settings");
              WebSerial.println("ap_ip_address updated via web settings");
          }
      } 
      if (request->hasParam(input_parameter24)) {
          //ap_subnet
          if ((request->getParam(input_parameter24)->value().length() > 0) && (request->getParam(input_parameter24)->value() != ap_subnet)) {
              inputMessage24 = request->getParam(input_parameter24)->value(); // assing the URLParam value
              ap_subnet = inputMessage24;
              if (ap_subnet.indexOf(";") != -1) {
                  WebSerial.println("f Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              if (String(inputMessage24) != ap_subnet) {
                  restartNeeded = true;
                  Serial.println("ap_subnet settings updated");
              }
              preferences.putString("ap_subnet", (String(ap_subnet).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(ap_subnet).c_str(),"apsubnet",millis()); // update the web page
              Serial.println("ap_subnet updated via web settings");
              WebSerial.println("ap_subnet updated via web settings");
          }
      } 
      if (request->hasParam(input_parameter25)) {
          //ap_gateway
          if ((request->getParam(input_parameter25)->value().length() > 0) && (request->getParam(input_parameter25)->value() != ap_gateway)) {
              inputMessage25 = request->getParam(input_parameter25)->value(); // assing the URLParam value
              if (String(inputMessage25) != ap_gateway) {
                  restartNeeded = true;
                  Serial.println("ap_gateway settings updated");
                  WebSerial.println("ap_gateway settings updated");
              }
              ap_gateway = inputMessage25;
              if (ap_gateway.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("ap_gateway", (String(ap_gateway).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(ap_gateway).c_str(),"apgateway",millis()); // update the web page
              Serial.println("ap_gateway updated via web settings");
              WebSerial.println("ap_gateway updated via web settings");
          }
      }
      if (request->hasParam(input_parameter26)) {
            //sta_ip_address
            if ((request->getParam(input_parameter26)->value().length() > 0) && (request->getParam(input_parameter26)->value() != sta_ip_address)) {
                inputMessage26 = request->getParam(input_parameter26)->value(); // assing the URLParam value
                if (String(inputMessage26) != ap_gateway) {
                    restartNeeded = true;
                    Serial.println("sta_ip_address settings updated");
                    WebSerial.println("sta_ip_address settings updated");
                }
                sta_ip_address = inputMessage26;
                if (sta_ip_address.indexOf(";") >= 0) {
                    WebSerial.println("h Script kiddiness detected, crashing the save paramater loop!");
                    Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                    return; // crash the loop and don't save the 1337 h4x0r script
                }
                preferences.putString("sta_ip_address", (String(sta_ip_address).c_str())); // add new setting to flash storage
                settings_saved = true;
                events.send(String(sta_ip_address).c_str(),"staipaddress",millis()); // update the web page
                Serial.println("sta_ip_address updated via web settings");
                WebSerial.println("sta_ip_address updated via web settings");
            }
        } 
        if (request->hasParam(input_parameter27)) {
            //sta_subnet
            if ((request->getParam(input_parameter27)->value().length() > 0) && (request->getParam(input_parameter27)->value() != sta_subnet)) {
                inputMessage27 = request->getParam(input_parameter27)->value(); // assing the URLParam value
                if (String(inputMessage27) != ap_gateway) {
                    restartNeeded = true;
                    Serial.println("sta_subnet settings updated");
                }
                sta_subnet = inputMessage27;
                if (sta_subnet.indexOf(";") != -1) {
                    WebSerial.println("i Script kiddiness detected, crashing the save paramater loop!");
                    Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                    return; // crash the loop and don't save the 1337 h4x0r script
                }
                preferences.putString("sta_subnet", (String(sta_subnet).c_str())); // add new setting to flash storage
                settings_saved = true;
                events.send(String(sta_subnet).c_str(),"stasubnet",millis()); // update the web page
                Serial.println("sta_subnet updated via web settings");
                WebSerial.println("sta_subnet updated via web settings");
            }
        } 
        if (request->hasParam(input_parameter28)) {
            //sta_gateway
            if ((request->getParam(input_parameter28)->value().length() > 0) && (request->getParam(input_parameter28)->value() != sta_gateway)) {
                inputMessage28 = request->getParam(input_parameter28)->value(); // assing the URLParam value
                if (String(inputMessage28) != sta_gateway) {
                    restartNeeded = true;
                    Serial.println("sta_gateway settings updated");
                }
                sta_gateway = inputMessage28;
                if (sta_gateway.indexOf(";") != -1) {
                    WebSerial.println("j Script kiddiness detected, crashing the save paramater loop!");
                    Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                    return; // crash the loop and don't save the 1337 h4x0r script
                }
                preferences.putString("sta_gateway", (String(sta_gateway).c_str())); // add new setting to flash storage
                settings_saved = true;
                events.send(String(sta_gateway).c_str(),"stasubnet",millis()); // update the web page
                Serial.println("sta_gateway updated via web settings");
                WebSerial.println("sta_gateway updated via web settings");
            }
        }  
      if (request->hasParam(input_parameter29)) {
          //udp_payload_en
          if ((request->getParam(input_parameter29)->value().length() > 0) && (request->getParam(input_parameter29)->value() != (String(udp_payload_en).c_str()))) {
              inputMessage29 = request->getParam(input_parameter29)->value(); // assing the URLParam value
              udp_payload_en = inputMessage29;
              preferences.putString("udp_payload_en", (String(udp_payload_en).c_str())); // add new setting to flash storage
              settings_saved = true;
              events.send(String(udp_payload_en).c_str(),"udppayloaden",millis()); // update the web page
              Serial.println("udp_payload_en updated via web settings");
              WebSerial.println("udp_payload_en updated via web settings");
          }
      }
      if (request->hasParam(input_parameter30)) {
          //sta_dns
          if (request->getParam(input_parameter30)->value() != sta_dnsL) {
              inputMessage30 = request->getParam(input_parameter30)->value(); // assing the URLParam value
              if (String(input_parameter30) != sta_dnsL) {
                  Serial.println("STA DNS settings updated");
                  WebSerial.println("STA DNS settings updated");
              }
              sta_dnsL = inputMessage30;
              if (sta_dnsL.indexOf(";") != -1) {
                  WebSerial.println("Script kiddiness detected, crashing the save paramater loop!");
                  Serial.println("Script kiddiness detected, crashing the save paramater loop!");
                  return; // crash the loop and don't save the 1337 h4x0r script
              }
              preferences.putString("sta_dns", (String(sta_dnsL).c_str())); // add new setting to flash storage
              settings_saved = true;
              restartNeeded = true;
              Serial.println("STA DNS updated via web settings");
              WebSerial.println("STA DNS updated via web settings");
          }
      }
      if (request->hasParam(input_parameter31) && (request->hasParam("ethsourcecheck"))) {
          //eth_enabled
          if (eth_enabled != 1) {
              eth_enabled = 1;
              restartNeeded = true;
              preferences.putBool("eth_enabled", 1); // add new setting to flash storage
              settings_saved = true;
              events.send("1","ethenabled",millis()); // update the web page
              Serial.println("Ethernet mode enabled");
              WebSerial.println("Ethernet mode enabled");
          } else {
              restartNeeded = false;
              Serial.println("Ethernet mode still enabled");
              WebSerial.println("Ethernet mode still enabled");
          }
      } else if (request->hasParam("ethsourcecheck")){
          eth_enabled = 0;
          restartNeeded = true;
          preferences.putBool("eth_enabled", 0); // add new setting to flash storage
          settings_saved = true;
          events.send("0","ethenabled",millis()); // update the web page
          Serial.println("ETH mode disabled");
          WebSerial.println("ETH mode disabled");
      }
      
      preferences.end();
      if (settings_saved) {
          Serial.println("Settings updated!");
          WebSerial.println("Settings updated!");
      } else {
          Serial.println("No settings updated!");
          WebSerial.println("No settings updated!");
      }
      
      request->send(LittleFS, "/response.html", String(), false, processor); // send the client the response 

      if (restartNeeded) {
          Serial.println("Restarting to apply new network settings!");
          WebSerial.println("Restarting to apply new network settings!");
          delay(250); // give serial buffer a chance to clear
          ESP.restart(); // restart the AE-CM
      } });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client)
                   {
      if(client->lastId()){
          Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
          WebSerial.print("Web client reconnected");
      }
      // send event with message "Hello, from AE-CM!", id current millis
      // and set reconnect delay to 1 second
      client->send("Hello, from AE-CM!", NULL, millis(), 10000); });

  server.on("/src/favicon.png", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to load favicon.ico file
    request->send(LittleFS, "/src/favicon.png", "image/png");                  // send the client the response
  });

  server.on("/src/styles.css", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to load style.css file
    AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/src/styles.css.gz", "text/css");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/src/bootstrap-icons.css", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to load bootstrap-icons.css file
    request->send(LittleFS, "/src/bootstrap-icons.css", "text/css");                   // send the client the response
  });

  server.on("/src/bootstrap.min.js", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to bootstrap.bundle.min.js file
    AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/src/bootstrap.min.js.gz", "text/javascript");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/src/bootstrap.min.js.map", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to bootstrap.bundle.min.js.map file
    AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/src/bootstrap.min.js.map.gz", "text/javascript");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/src/bootstrap-icons.woff", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to bootstrap-icons.woff file
    request->send(LittleFS, "/src/bootstrap-icons.woff", "font/woff");                  // send the client the response
  });

  server.on("/src/bootstrap-icons.woff2", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to bootstrap-icons.woff2 file
    request->send(LittleFS, "/src/bootstrap-icons.woff2", "font/woff2");                 // send the client the response
  });

  server.on("/src/ae_black_64.png", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to ae_black_64.png file
    request->send(LittleFS, "/src/ae_black_64.png", "image/png");                  // send the client the response
  });

  server.on("/relay1on", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to set GPIO to HIGH
    digitalWrite(RELAY_1_ON, HIGH);
    relay1_state = "ON";
    events.send(String(relay1_state).c_str(), "relay1state", millis()); // update the web page
    request->send(200, "text/plain", "ok");                             // send the client the response
  });

  server.on("/relay1off", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to set GPIO to LOW
    digitalWrite(RELAY_1_ON, LOW);
    relay1_state = "OFF";
    events.send(String(relay1_state).c_str(), "relay1state", millis()); // update the web page
    request->send(200, "text/plain", "ok");                             // send the client the response
  });

  server.on("/relay2on", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to set GPIO to HIGH
    digitalWrite(RELAY_2_ON, HIGH);
    relay2_state = "ON";
    events.send(String(relay2_state).c_str(), "relay2state", millis()); // update the web page
    request->send(200, "text/plain", "ok");                             // send the client the response
  });

  server.on("/relay2off", HTTP_GET, [](AsyncWebServerRequest *request) { // Route to set GPIO to LOW
    digitalWrite(RELAY_2_ON, LOW);
    relay2_state = "OFF";
    events.send(String(relay2_state).c_str(), "relay2state", millis()); // update the web page
    request->send(200, "text/plain", "ok");                             // send the client the response
  });

  WebSerial.begin(&server); // webserial socket, available at /webserial. Used for debugging like UART

  ElegantOTA.begin(&server, http_username.c_str(), http_password.c_str()); // Start ElegantOTA server, require auth
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  if (eth_enabled)
  {
    Serial.print("Ethernet OTA and MDNS broadcaster started on: "); // just an FYI
    Serial.println(ETH.localIP());                                  // just an FYI
  }
  if (ap_enabled)
  {
    Serial.print("WiFi AP OTA and MDNS broadcaster started on: ");    // just an FYI
    Serial.println(WiFi.softAPIP());                                  // just an FYI
  }
  if (sta_enabled)
  { 
    Serial.print("WiFi STA and MDNS broadcaster started on: ");       // just an FYI
    Serial.println(WiFi.localIP());                                   // just an FYI
  }
  
  server.onNotFound(notFound); // deals with 404s
  server.addHandler(&events);  // adds ebent handling (callbacks)
  server.begin();              // start the webserver, after all else is started

  Serial.print("Ethernet Web server started on: "); // just an FYI
  Serial.println(ETH.localIP());                    // just an FYI
  Serial.print("WiFi Web server started on: ");     // just an FYI
  Serial.println(WiFi.localIP());                   // just an FYI

  while (myI2CGPS.begin() == false)
  {
    Serial.println("GPS module failed to respond!!!");
    WebSerial.println("GPS module failed to respond!!!");
  }
  Serial.println("GPS module found!");
  WebSerial.println("GPS module found!");

  if (!eth_connected)
  {
    Serial.println("ETH Disconnected!!!\n");    // should only see this if an Ethernet cable isn't connected
    WebSerial.println("ETH Disconnected!!!\n"); // should only see this if an Ethernet cable isn't connected
  }
  
  MQTT_connect(); // connect to Adafruit.io
}

// runs after setup and loops forever
void loop()
{
  ElegantOTA.loop();
  getSensorDataTimer.run();  // runs the getSensorData function on a timer (of 10 seconds) //ToDo: make the time configurable
  updateMDNSDataTimer.run(); // updates the MDNS data on a timer (of 10 minutes)           //ToDo: make the time configurable
  mrd->loop();               // keeps track of the multi reset
  check_status();            // keeps track of the multi reset timeout
  esp_task_wdt_reset();      // ticks the watchdog

  if (((millis() - previousTime) > MQTT_KEEP_ALIVE * 1000) || (firstBoot)) 
  {
    if (WiFi.status() == WL_CONNECTED)
    { // publish data every 5 mins
      if (!BOSS_CM_B.publish(ch0_voltage))
      {
        Serial.println(F("Publishing Battery Voltage Failed!"));
      }
      else
      {
        Serial.println(F("Published Battery Voltage!"));
      }

      if (!BOSS_CM_S.publish(ch1_voltage))
      {
        Serial.println(F("Publishing Solar Voltage Failed!"));
      }
      else
      {
        Serial.println(F("Published Solar Voltage!"));
      }
      if (!BOSS_CM_C.publish(ch2_voltage))
      {
        Serial.println(F("Publishing Car Voltage Failed!"));
      }
      else
      {
        Serial.println(F("Published Car Voltage!"));
      }
      if (!BOSS_CM_T.publish(ch0_temperature))
      {
        Serial.println(F("Publishing Temperature] Failed!"));
      }
      else
      {
        Serial.println(F("Published Temperature Voltage!"));
      }
      
      firstBoot = false;
    }
    else 
    {
      WiFi.disconnect();
      WiFi.reconnect();
    }
    previousTime = millis();
  }
}