/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <Wifi.h>
void setup()
{
  // initialize serial port for communication
  Serial.begin(115200);

  // display a blank line
  Serial.println();
  Serial.println();

  // get the MAC address of ESP8266 and display it
  String macAddress = WiFi.macAddress();
  Serial.print("MAC address = ");
  Serial.println(macAddress);
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(400);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  delay(100);
}
