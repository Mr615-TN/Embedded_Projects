/*  Program:        Lab04
**  Author:         John Hutton
**  Created Date:   02/17/23
**  Modified Date:  02/14/24
**
**  Description
**  -----------
**  This lab wires four buttons tied to for different priority tasks.
**  We implement a round robin and a function queue method
**  for running these tasks.
**
**  Reference Links
**  ---------------
**  https://github.com/thomasfredericks/Bounce2
*/
#include <Arduino.h>
// #include <esp.h>
// #include <Wifi.h>
#include <Bounce2.h> // to debounce pushbuttons
#include <string.h>

/* Prototype Functions */
// void dummyFunction();
void RoundRobin();
void FunctionQueue();
void initializeButtons();
void checkButtons();
void SimpleDebug();
void displayMenu();
void helpMenu();
void processCommand(String command);
void setTimes(String command);

/* End Prototype function */

/* Program Globals*/
#define DEBOUNCE_INTERVAL 40
#define AButtonPin 21 // ESP32 IO21
#define BButtonPin 17 // ESP32 IO17
#define CButtonPin 16 // ESP32 IO16
#define DButtonPin 19 // ESP32 MISO
#define UpdateInt 5000

enum DeviceState {
  BLOCKED,
  READY,
  RUNNING,
};

struct Device {
  DeviceState state;
  unsigned long serviceTime;
  unsigned long waitTime;
  unsigned long runTime;
  Bounce2::Button *button;
};

// Variables

Device buttons[4];
bool isRoundRobin = true; // Hopefully defaults to Round Robin
int currentDevice = 0;
String inputBuffer = "";

Bounce2::Button A_Button = Bounce2::Button();
Bounce2::Button B_Button = Bounce2::Button();
Bounce2::Button C_Button = Bounce2::Button();
Bounce2::Button D_Button = Bounce2::Button();

// setup
void setup() {
  // Serial terminal for debug and interaction
  Serial.begin(115200);
  Serial.print("\nSerial ready!\n");
  A_Button.attach(AButtonPin, INPUT_PULLUP);
  A_Button.interval(5);
  B_Button.attach(BButtonPin, INPUT_PULLUP);
  B_Button.interval(5);
  C_Button.attach(CButtonPin, INPUT_PULLUP);
  C_Button.interval(5);
  D_Button.attach(DButtonPin, INPUT_PULLUP);
  D_Button.interval(5);

  initializeButtons();
  displayMenu();
  Serial.print(">");
}

void loop() {
  while (Serial.available() > 0) {
    char c = tolower(Serial.read());
    Serial.write(c); // Echo the character back

    if (c == '\r' || c == '\n') {
      Serial.println();
      processCommand(inputBuffer);
      inputBuffer = "";
      Serial.print(">");
    } else {
      inputBuffer += c;
    }
  }
  if (isRoundRobin) {
    RoundRobin();
  } else {
    FunctionQueue();
  }
  // SimpleDebug();
}

// Functions
// void dummyFunction() {
//   // This is just a placeholder
//   return;
// }

void SimpleDebug() {
  A_Button.update();
  if (A_Button.pressed()) {
    Serial.println("A Button pressed");
  }
  B_Button.update();
  if (B_Button.pressed()) {
    Serial.println("B Button pressed");
  }
  C_Button.update();
  if (C_Button.pressed()) {
    Serial.println("C Button pressed");
  }
  D_Button.update();
  if (D_Button.pressed()) {
    Serial.println("D Button pressed");
  }
}

void initializeButtons() {
  buttons[0].state = BLOCKED;
  buttons[0].serviceTime = 0;
  buttons[0].button = &A_Button;

  buttons[1].state = BLOCKED;
  buttons[1].serviceTime = 0;
  buttons[1].button = &B_Button;

  buttons[2].state = BLOCKED;
  buttons[2].serviceTime = 0;
  buttons[2].button = &C_Button;

  buttons[3].state = BLOCKED;
  buttons[3].serviceTime = 0;
  buttons[3].button = &D_Button;
}

void checkButtons() {
  for (int i = 0; i < 4; i++) {
    buttons[i].button->update();
    if (buttons[i].button->pressed()) {
      if (buttons[i].state == BLOCKED && buttons[i].serviceTime > 0) {
        buttons[i].state = READY;
        buttons[i].waitTime = millis();
        Serial.print("Device ");
        Serial.print((char)('A' + i));
        Serial.println(" is now ready");
      } else if (buttons[i].serviceTime == 0) {
        Serial.print("Device ");
        Serial.print((char)('A' + i));
        Serial.println(" ignored - service time is now 0");
      }
    }
  }
}
void RoundRobin() {
  checkButtons();

  for (int i = 0; i < 4; i++) {
    if (buttons[i].state == RUNNING) {
      if (millis() >= buttons[i].runTime + buttons[i].serviceTime) {
        // Calculate and display response time
        unsigned long responseTime = (millis() - buttons[i].waitTime);
        Serial.print("Device ");
        Serial.print((char)('A' + i));
        Serial.print(" response time: ");
        Serial.print(responseTime);
        Serial.println("ms");

        buttons[i].state = BLOCKED;
        Serial.print("Device ");
        Serial.print((char)('A' + i));
        Serial.println(" is now Blocked");
      }
    }
  }

  // Find next device to run
  bool foundNext = false;
  for (int i = 0; i < 4; i++) {
    int deviceIndex = (currentDevice + i) % 4;
    if (buttons[deviceIndex].state == READY) {
      bool otherRunning = false;
      for (int j = 0; j < 4; j++) {
        if (buttons[j].state == RUNNING) {
          otherRunning = true;
          break;
        }
      }

      if (!otherRunning) {
        buttons[deviceIndex].state = RUNNING;
        buttons[deviceIndex].runTime = millis();
        Serial.print("Device ");
        Serial.print((char)('A' + deviceIndex));
        Serial.println(" is now Running");
        currentDevice = (deviceIndex + 1) % 4;
        foundNext = true;
        break;
      }
    }
  }

  static bool wasAllBlocked = false;
  if (!foundNext) {
    bool allBlocked = true;
    for (int i = 0; i < 4; i++) {
      if (buttons[i].state != BLOCKED) {
        allBlocked = false;
        break;
      }
    }
    if (allBlocked && !wasAllBlocked) {
      Serial.println("All devices blocked");
    }
    wasAllBlocked = allBlocked;
  } else {
    wasAllBlocked = false;
  }
}

void FunctionQueue() {
  checkButtons();

  for (int i = 0; i < 4; i++) {
    if (buttons[i].state == RUNNING) {
      if (millis() >= buttons[i].runTime + buttons[i].serviceTime) {
        unsigned long responseTime = (millis() - buttons[i].waitTime);
        Serial.print("Device ");
        Serial.print((char)('A' + i));
        Serial.print(" response time: ");
        Serial.print(responseTime);
        Serial.println("ms");

        buttons[i].state = BLOCKED;
        Serial.print("Device ");
        Serial.print((char)('A' + i));
        Serial.println(" is now Blocked");
      }
    }
  }

  bool foundTask = false;
  bool otherRunning = false;

  for (int i = 0; i < 4; i++) {
    if (buttons[i].state == RUNNING) {
      otherRunning = true;
      break;
    }
  }

  if (!otherRunning) {
    for (int i = 0; i < 4; i++) {
      if (buttons[i].state == READY) {
        buttons[i].state = RUNNING;
        buttons[i].runTime = millis();
        Serial.print("Device ");
        Serial.print((char)('A' + i));
        Serial.println(" is now Running");
        foundTask = true;
        break;
      }
    }
  }

  static bool wasAllBlocked = false;
  if (!foundTask && !otherRunning) {
    bool allBlocked = true;
    for (int i = 0; i < 4; i++) {
      if (buttons[i].state != BLOCKED) {
        allBlocked = false;
        break;
      }
    }
    if (allBlocked && !wasAllBlocked) {
      Serial.println("All devices blocked");
    }
    wasAllBlocked = allBlocked;
  } else {
    wasAllBlocked = false;
  }
}

void processCommand(String command) {
  command.trim();
  if (command == "fq") {
    isRoundRobin = false;
    Serial.println("Switched to Function Queue Scheduling");
  } else if (command == "rr") {
    isRoundRobin = true;
    Serial.println("Switched to Round Robin Scheduling");
  } else if (command == "show") {
    Serial.print("Current Scheduler: ");
    Serial.println(isRoundRobin ? "Round Robin" : "Function Queue");
    Serial.println("Service Times:");
    for (int i = 0; i < 4; i++) {
      Serial.print("Device ");
      Serial.print((char)('A' + i));
      Serial.print(": ");
      Serial.print(buttons[i].serviceTime);
      Serial.println("ms");
    }
  } else if (command == "reset") {
    for (int i = 0; i < 4; i++) {
      buttons[i].serviceTime = 0;
      buttons[i].state = BLOCKED;
    }
    Serial.println("All service times reset to 0");
  } else if (command == "help") {
    helpMenu();
  } else if (command.length() > 0) {
    setTimes(command);
  }
}

void setTimes(String command) {
  char *str = (char *)command.c_str();
  char *str_token = strtok(str, " ");

  while (str_token != NULL) {
    char device = str_token[0];
    str_token = strtok(NULL, " ");

    if (str_token != NULL) {
      int time = atoi(str_token);
      int index = tolower(device) - 'a';

      if (index >= 0 && index < 4) {
        buttons[index].serviceTime = time;
        Serial.print("Device ");
        Serial.print((char)('A' + index));
        Serial.print(" service time set to ");
        Serial.print(time);
        Serial.println("ms");
      }
    }
    str_token = strtok(NULL, " ");
  }
}
void displayMenu() {
  Serial.println("Welcome to Task Scheduler");
  Serial.println("Here is the list of commands:");
  Serial.println("fq");
  Serial.println("rr");
  Serial.println("show");
  Serial.println("reset");
  Serial.println("help");
}

void helpMenu() {
  Serial.println("fq -> Set the simulation to Function Queue Scheduling");
  Serial.println("rr -> Set the simulation to Round Robin scheduling");
  Serial.println(
      "show -> Show the scheduler type and service times for all four tasks");
  Serial.println("reset -> Reset all service times to zero (the default)");
  Serial.println("help -> Show the display menu again with the commands");
}
