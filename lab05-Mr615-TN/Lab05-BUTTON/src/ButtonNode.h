// This file accompanies the mqtt_btnNode_starter_code.ino v2.0 program
#include <Arduino.h>
// system defines
#define DEBOUNCE_INTERVAL 10  // 5mS works well for circuit-mount PBs
#define PB_ON 21   // pin connected to led "on" switch (ESP32 IO21/Pin 21)
#define PB_OFF 17  // pin connected to led "off" switch (ESP32 IO17/Pin 17)

// uncomment for Lipscomb broker, comment out for "brokerX"
// #define LIPSCOMB
#define ETHERNET
// #define HUTTONHOME

// Network and MQTT broker credentials
#ifdef LIPSCOMB
const char* mqttBroker = "10.51.97.101";  // ECE mosquitto server (wlan0)
const char* ssid = "LipscombGuest";  // no PW needed for Lipscomb guest wifi
#elif defined(ETHERNET)
const char* mqttBroker = "10.200.97.100";  // Wired ECE mosquitto server (eth0)
const char* ssid = "LipscombGuest";        // same as straight WiFi
#elif defined(HUTTONHOME)
const char* mqttBroker = "192.168.0.251";  // address of brokerX
const char* ssid = "HuttonWireless2-4G";   // ssid of brokerX's network
const char* password = "Testing01";        // password for brokerX's network
#else
const char* mqttBroker = "1.1.1.1";
const char* ssid = "";
const char* password = "password";  // password for brokerX's network
#endif

int mqttPort = 1883;

String buttonClientID = "btnNode03";
String ledClientID = "ledNode03";
String cmdOn = "on";
String cmdOff = "off";