#include <Arduino.h>
// #include <esp.h>
//  #include <Wifi.h>
#include <string.h>

#include "Adafruit_Si7021.h"
// #include "Adafruit_HTU21DF.h"
#include <LiquidCrystal_I2C.h>

// Globals

// Note:  lcd.init() calls the Wire.h builtin that uses the default
// I2C bus for the ESP32.  Defines below show should show these pin numbers!
// For other boards, there may be a way to pre-initilize the
// Wire object with this data, but seems to be working with defaults.
#define SCL 22 // Use default SCL for Huzzah32
#define SDA 23 // Use default SDA for Huzzah32
// This is the default address for the 7021 Sensor.
#define SensorI2CAddress 0x40
Adafruit_Si7021 sensor = Adafruit_Si7021();
// This is the address for the LCD display (can vary).
#define LCDI2CAddress 0x27
// Setup for 16 columns and 2 rows.
LiquidCrystal_I2C lcd(LCDI2CAddress, 16, 2);

// Sensor variables
float humidity = 0;
float temp = 0;
// LCD variables - Use sprintf to build yor print string.
// LCD only has 16 columns for characters.
char tempStr[16];
char humidityStr[16];
unsigned long msCurrent, msLast; // for capturing MS for timing
#define msUpdateInt 5000         // Update the readings every msUpdateInt

// Prototype Functions
// Section needed if you write your function code below the calling
// locations.
void simpleExample();
void findI2CAddresses();
float readTemp();
float readHumidity();
void displayMenu();
void autoRefresh();

void setup() {
  // Setup a serial terminal for debugging
  Serial.begin(115200);
  // wait for serial port to open
  while (!Serial) {
    delay(10);
  }
  Serial.println("\nSerial ready!");

  // Helper code to scan I2C addresses.  Only run for Debug.
  // Must comment out to run any additional code!
  // findI2CAddresses();
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021!");
    while (1)
      ;
  }
  Serial.println("Found Si7021!");
  // Initialize the LCD
  lcd.init();
  // Turn on the blacklight and print a message.
  lcd.backlight();
  // lcd.setCursor(0, 0);
  // lcd.print("Test");
  lcd.clear();

  // Timing -  One way to track time for 5s delay.
  msCurrent = millis();
  msLast = msCurrent;
  displayMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char choice = Serial.read();
    Serial.write(choice);
    Serial.println();

    float temp;
    float humidity;

    lcd.clear();

    switch (choice) {
    case 't':
    case 'T':
      temp = readTemp();
      Serial.print("Temperature: ");
      Serial.print(temp);
      Serial.print(" \xDF"); // Degrees symbol
      Serial.println("F");
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.write(223); // Degrees symbol for LCD
      lcd.print("F");
      break;

    case 'h':
    case 'H':
      humidity = readHumidity();
      Serial.print("Relative humidity: ");
      Serial.print(humidity);
      Serial.println("%");
      lcd.setCursor(0, 0);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");
      break;

    case 'b':
    case 'B':
      temp = readTemp();
      humidity = readHumidity();
      Serial.print("Temperature: ");
      Serial.print(temp);
      Serial.print(" \xDF");
      Serial.print("F, Relative humidity: ");
      Serial.print(humidity);
      Serial.println("%");
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.write(223);
      lcd.print("F");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");
      break;

    case '?':
      displayMenu();
      break;

    case 'x':
    case 'X':
      Serial.println("Goodbye!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Goodbye!");
      delay(2000);
      lcd.clear(); 
      while (1) {
        // Infinite loop
      }
      break;

    default:
      displayMenu();
      break;
    }

    if (choice != 'x' && choice != 'X') {
      Serial.print("Choice? ");
    }
  }
  autoRefresh();
}

// Function Declarations
// To make your code cleaner, add your functions here to keep your
// setup() and loop() code more contained.
// Declaring functions below the calling locatin requires the
// Prototype section to be filled out above.

void simpleExample() {
  // function does nothing
}

// ProfHutton - Debugging functions
void si7021_print_details() {
  Serial.print("Found model ");
  switch (sensor.getModel()) {
  case SI_Engineering_Samples:
    Serial.print("SI engineering samples");
    break;
  case SI_7013:
    Serial.print("Si7013");
    break;
  case SI_7020:
    Serial.print("Si7020");
    break;
  case SI_7021:
    Serial.print("Si7021");
    break;
  case SI_UNKNOWN:
  default:
    Serial.print("Unknown");
  }
  Serial.print(" Rev(");
  Serial.print(sensor.getRevision());
  Serial.print(")");
  Serial.print(" Serial #");
  Serial.print(sensor.sernum_a, HEX);
  Serial.println(sensor.sernum_b, HEX);
}

void findI2CAddresses() {
  // Leveraged from
  // https://learn.adafruit.com/scanning-i2c-addresses/arduino?gad_source=1&gclid=Cj0KCQiAn-2tBhDVARIsAGmStVlEmMDNCvjza_EptvBgsr6ucvNK5ZlaAZ7JAfS3WR0jNmUUabE9-ecaArEgEALw_wcB
  byte error, address;
  int nDevices;

  Serial.println("\nI2C Scanner");

  Wire.begin();

  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }

  // End with eternal spin loop
  Serial.println("Ending with eternal spin loop...");
  while (1) {
    delay(10000);
    delay(2000);
  }
}

// Need to figure out what is going wrong here and why it is not printing the
// temperature and humidity, but just NAN
float readTemp() {
  float tempC = sensor.readTemperature();
  float tempF = tempC * 1.8 + 32;
  return tempF;
}
float readHumidity() {
  float humid = sensor.readHumidity();
  return humid;
}

void displayMenu() {
  Serial.println("Welcome to ETHOS");
  Serial.println("The Electronic Temperature & Humidity Observation System");
  Serial.println("Your options:");
  Serial.println("T Request temperature");
  Serial.println("H Request relative humidity");
  Serial.println("B Request both temperature and relative humidity");
  Serial.println("? Help (shows this menu)");
  Serial.println("X Exit");
  Serial.print("Choice? ");
}

void autoRefresh() {
  msCurrent = millis();
  if (msCurrent - msLast >= msUpdateInt) {
    temp = readTemp();
    humidity = readHumidity();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.write(223); // Degrees symbol
    lcd.print("F");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print("%");
    msLast = msCurrent;
  }
}
