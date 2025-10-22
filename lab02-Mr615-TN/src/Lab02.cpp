/*
 * Author:  John Hutton
 * Modified Date:  01/13/23
 * 
 * Code to control an H-Bridge to run a DC Fan
 */

// Includes
#include <Arduino.h>  // Required for Arduino style setup() loop() environment
//#include <ESP.h>      // Required for basic ESP32 board definitions

// Variables and Defines
//#define lint long long int
#define FanHigh 5
#define FanLow  4

#define FanMax 255
#define FanHalf 128
#define FanGnd 30

#define ReverseButton 12
#define SpeedButton 27

int buttonState_1 = 0;
int buttonState_2 = 0;
int oldButtonState_1 = HIGH;
int oldButtonState_2 = HIGH;
long totalDebounceTime = 0;
long totalDebounceDelay = 50;
int input_button1 = 0;
int input_button2 = 0;
int state = 0;
// Prototype functions


/* NOTE - Prof Hutton
   I generally like the setup() and loop() at the very top after
   the variable definitions.  This means I need to have prototype
   functions declared here so they are visible.  Simply keep the
   first line of the function definition (must match real function below)
     void myFunction (int myVar);
   It is OK to have setup() and loop() ath the bottom, then 
   prototype functions are not needed.
*/
void clockwise(int FanSpeed);
void counterclockwise(int FanSpeed);
void checkReverseButton();
void checkSpeedButton();

// Arduino Setup - Runs once
void setup()
{
  // Initalize the pins to drive the H-bridge
   pinMode(FanHigh, OUTPUT);
   pinMode(FanLow, OUTPUT); 

  // Initalize the button pins
   pinMode(ReverseButton, INPUT_PULLUP);
   pinMode(SpeedButton, INPUT_PULLUP);

  // initialize serial port for communication
  /* NOTE - Prof Hutton
     Serial terminal is very useful for debug.  Be sure
     the speed here matches monitor_speed in the 
     platformio.ini
  */
   Serial.begin(115200);
   Serial.println();
   Serial.println("Serial Ready!");
}

// Arduino Loop - Runs infinately
void loop()
{
   checkReverseButton();
   checkSpeedButton();
   switch(state) {
      case 1:
         clockwise(FanGnd);
         break;
      case 2:
         clockwise(FanHalf);
         break;
      case 3:
         clockwise(FanMax);
         break;
      case 4:
         counterclockwise(FanGnd);
         break;
      case 5:
         counterclockwise(FanHalf);
      case 6:
         counterclockwise(FanMax);
      default:
         clockwise(0);
   }

  /* NOTE - Prof Hutton
     Think through what your main loop must do
     (check buttons with debounce, control fan speed
     and direction, reset when both buttons are pressed)
     I strongly recomend keeping your loop short and using
     functions!  This facilitates readability and 
     testability.
     ALSO, design/code/test.  Get the smallest things possible
     working, get all the pieces working, then combine 
     everything together
  */

}

void checkReverseButton() {
   input_button1 = digitalRead(ReverseButton);
   if (input_button1 != oldButtonState_1) {
      totalDebounceTime = millis();
      while ((millis() - totalDebounceTime) < totalDebounceDelay) {
         input_button1 = digitalRead(ReverseButton);
      }
      if (input_button1 != oldButtonState_1) {
         if (input_button1 == HIGH) {
            Serial.println("Reverse Button Pressed");
            state += 3;
            if (state > 6) {state -= 6;}
         } else {
            Serial.println("Reverse Button Not Pressed");
         }
         oldButtonState_1 = input_button1;
      }
   }
}

void checkSpeedButton() {
   input_button2 = digitalRead(SpeedButton);
   if (input_button2 != oldButtonState_2) {
      totalDebounceTime = millis();
      while ((millis() - totalDebounceTime) < totalDebounceDelay) {
         input_button2 = digitalRead(SpeedButton);
      }
      if (input_button2 != oldButtonState_2) {
         if (input_button2 == HIGH) {
            Serial.println("Speed Button Pressed");
            state = (state % 3) + 1;
            if (state == 4) {state = 1;}
         } else {
            Serial.println("Speed Button Not Pressed");
         }
         oldButtonState_2 = input_button2;
      }
   }
}

void clockwise(int FanSpeed) {
   analogWrite(FanHigh, 0);
   analogWrite(FanLow, FanSpeed);
}



void counterclockwise(int FanSpeed) {
   analogWrite(FanHigh, FanSpeed);
   analogWrite(FanLow, 0);
}


// Support Functions
/* NOTE - Prof Hutton
   See note above.  If you put the setup() and loop()
   last, they will come below all your functions.
*/
