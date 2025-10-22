// This .h file accompanies the mqtt_ledNode.ino v2.1 program.
#include <Arduino.h>
// system defines
#define LED 21  // active high LED, needs current limiting resistor
#define ON 1
#define OFF 0

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

// Client ID of this LED controller
String buttonClientID = "btnNode03";  // Change XX to your two-digit ID

// ID of the remote LED node
String ledClientID = "ledNode03";  // Change XX to your two-digit ID
//compileErrorHere();                // Remove this when Node names fixed.

// Commands
String cmdOn = "on";
String cmdOff = "off";