#include <ESP8266WiFi.h>

// Program: MAC.ino
// Name: <your name here>
// Date: <submission date here>
// This program prints the MAC address of an ESP8266 module.

void setup() {
  // initialize serial port for communication
  Serial.begin(115200);

  // display a blank line
  Serial.println();
  Serial.println();

  // get the MAC address of ESP8266 and display it
  String macAddress = WiFi.macAddress();
  Serial.print("MAC address = ");
  Serial.println(macAddress);
}

void loop() {
}
