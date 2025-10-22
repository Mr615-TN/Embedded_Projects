#include <Arduino.h>
// #include <esp.h>
// #include <Wifi.h>
#include "Adafruit_HTU21DF.h"
#include "Adafruit_Si7021.h"
#include <LiquidCrystal_I2C.h>
#include <string.h>

// Globals

// Note:  lcd.init() calls the Wire.h builtin that uses the default
// I2C bus for the ESP32.  Defines below show should show these pin numbers!
// For other boards, there may be a way to pre-initilize the
// Wire object with this data, but seems to be working with defaults.
#define SCL 22 // Use default SCL for Huzzah32
#define SDA 23 // Use default SDA for Huzzah32
#define SensorI2CAddress                                                       \
  0x40 // This is the default address for the 7021 and HTU21DF Sensor.

// Support for 2 sensor boards:  Si7021(5-pin with unpined 3.3V), HTU21D (4-pin
// smaller board) If you have the HTU21DF part, uncomment the next #define line!
// #define HTU21DF true
#ifndef HTU21DF
Adafruit_Si7021 sensor = Adafruit_Si7021();
#else
Adafruit_HTU21DF sensor = Adafruit_HTU21DF();
#endif

// Sensor variables
float humidity = 0;
float temp = 0;

// Sleep Time  Need to figure out proper time
#define deepSleepTime 10000000 // should put the esp32 board to sleep for 10 seconds
// fix_above_line_then_delete_this_line_compile_error();

// Timing variables
unsigned long msCurrent, msNext, msLast; // for simple timing
int dotCount = 0;
#define DOT_INTERVAL 100
// Prototype Functions
void dummyPrototypeFunction();
float readTemp();
float readHumidity();
void displayDots();
void enterDeepSleep();

void setup() {
  Serial.begin(115200);
  // Serial.begin(74800);
  //  wait for serial port to open
  while (!Serial) {
    delay(10);
  }
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021!");
    while (1)
      ;
  }
  Serial.println("Found Si7021!");
  Serial.println("I'm awake!");

  // Prep the sleep routine
  Serial.print("I'm getting sleepy");

  // Display on serial with minimal characters
  Serial.print("Temperature: ");
  Serial.print(temp, 1);
  Serial.print(" \xDF"); // Degrees symbol
  Serial.println("F");
  Serial.print("Relative humidity: ");
  Serial.print(humidity, 1);
  Serial.println("%");

  // Prepare for sleep sequence
  Serial.println("I'm getting sleepy");

  // Initialize timing variables
  msCurrent = millis();
  msLast = msCurrent;
  msNext = msCurrent + DOT_INTERVAL;
  dotCount = 0;
}

void loop() { 
  displayDots();
}

void dummyProtoFunction() {
  // Dummy function
}

float readTemp() {
  float tempC = sensor.readTemperature();
  float tempF = tempC * 1.8 + 32;
  return tempF;
}
float readHumidity() {
  float humid = sensor.readHumidity();
  return humid;
}

void displayDots() {
  msCurrent = millis();
  
  // Print a dot every 100ms
  if (msCurrent >= msNext && dotCount < 50) {
    Serial.print(".");
    dotCount++;
    msNext = msCurrent + DOT_INTERVAL;
  }
  
  // After 50 dots (5 seconds), go to sleep
  if (dotCount >= 50) {
    Serial.println("\nGood night!");
    
    // Small delay to ensure the message is sent
    delay(100);
    
    enterDeepSleep();
  }
}

void enterDeepSleep() {
  // Configure the timer to wake up the ESP32
  esp_sleep_enable_timer_wakeup(deepSleepTime);
  
  // Enter deep sleep mode
  esp_deep_sleep_start();
  
  // Code after this point will not be executed
  // The ESP32 will restart and run setup() after waking up
}