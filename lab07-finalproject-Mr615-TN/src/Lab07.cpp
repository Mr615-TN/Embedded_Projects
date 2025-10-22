/*************************************************************************************
 * Sandbox for developing MQTT apps at Lipscomb
 ************************************************************************************/

/*************************************************************************************
 *     Program name: Lab07.cpp
 *          Version: 2.2
 *             Date: Feb 11, 2017
 *           Author: Greg Nordstrom
 *         Modified: John Hutton
 *             Date: Mar 26, 2025
 *         Platform: ESP32
 *    Additional HW: Buzzer, 7-seg LED, 74HC595, rotary encoder w/ button, 220
 *                   ohm resistors for 7-seg LED.
 * Additional files: Lab07.h
 * Req'd libraries: WiFi, PubSubClient, ArduinoJson, Bounce2, Rotary
 *
 * This program implements a simple communication network consisting of up to 16
 * Huzzah32 Feather with an ESP32 processors. Nodes communicate via an MQTT
 * broker (typically a RPi running Mosquitto) to send heartbeat messages as well
 * as "ringring" messages. If a ringring message is received from a valid node
 * (0-16), the receiver plays a brief dual-tone sound using a local buzzer.
 *Nodes must also respond to "topics" messages by sending a "registeredFor"
 *message listing all the topics for which it is registered. These are listed
 *below:
 *
 * Modified for ESP32 board (fundamentally the same as original ESP8266 design)
 *
 * Received messages:
 *    Topic: "ece/node01/topics"
 *    Usage: To request a list of all topics this node is registered for
 *  Payload: none
 *
 * Emitted messages:
 *    Topic: "ece/node01/registeredFor"
 *    Usage: Reply with all N topics this node is registered for
 *  Payload: {"NodeName":"node01", "Topic0":"topic0",...,"TopicN-1":"topicN-1"}
 *
 *    Topic: "ece/node01/ringring"
 *    Usage: Sends a ringring request to other nodes
 *  Payload: {"srcNode":"nodess","dstNode":"nodedd"}
 *           where ss and dd are source and destination node numbers,
 *           respectively
 *
 * This program uses the public domain PubSubClient library to perform MQTT
 * messaging functions, and all message payloads are encoded in JSON format.
 * IMPORTANT NOTES ON PubSubClient MQTT library for Arduino:
 * 1. Default message size, including header, is only 128 bytes (rather small).
 * 2. Increase/decrease by changing MQTT_MAX_PACKET_SIZE inside PubSubClient.h.
 * 3. As of 6/14/16, this value is set to 512 (in PubSubClient.h), allowing 7-10
 *    topics to be displayed (depends on individual topic lengths of course).
 *
 * This code does not use interrupts for pushbutton processing (a fool's errand
 * when the switches are not hardware debounced). Instead, each button is
 * debounced by a Bounce2 object.
 *
 * This node responds to topics messages by sending a registeredFor message:
 *
 *    Topic: "ece/node01/registeredFor"
 *    Usage: Reply with all N topics this node is registered for
 *  Payload: {"NodeName":"node01", "Topic0":"topic0",...,"TopicN-1":"topicN-1"}
 *
 * It should be noted that this node's functionality can be fully exercised
 * using an MQTT sniffing program such as MQTT-Spy (avaliable on GitHub).
 * MQTT-Spy is a Java program, so Java must be installed on the host machine.
 * MQTT-spy is invoked on a Windows machine from the command line as follows:
 *
 *     C:>java -jar mqtt-spy-x.x.x-jar-with-dependencies.jar
 *
 * where x.x.x is the current version of MQTT.
 *
 * Other tools that may be used instead are MQTT Explorer or MQTTX.
 *
 * Modification Notes:
 * - Lab05 had similar modifications...
 * - Pins have been adapted for the ESP32
 * - Libraries have been adapted for the ESP32
 * - The rotary encoder library has been changed to ESP32Encoder
 * - The tone library has been changed to ToneESP32
 * - The ArduinoJson has been upgraded to v7 (minor changes)
 *
 ************************************************************************************/
// included configuration file and support libraries
#include <Arduino.h>
#include <ArduinoJson.h> // for encoding/decoding MQTT payloads in JSON format
#include <Bounce2.h> // debounce pushbutton (by Thomas O Frederics vTBD version)
#include <ESP32Encoder.h> // New libary (in arduino libraries) for rotary encoder
#include <Esp.h>          // Esp32 support
#include <PubSubClient.h> // MQTT client (by Nick O'Leary?? vTBD version)
#include <WiFi.h>         // wi-fi support
                          // Slightly simpler to implement
                          // https://github.com/madhephaestus/ESP32Encoder
#include <ToneESP32.h> // Dedicated tone library for ESP32 (gets rid of ledc errors)

#include "Lab07.h" // included in this project

WiFiClient wfClient;
PubSubClient psClient(wfClient);
bool MQTTConnected;

Bounce2::Button button = Bounce2::Button();
ESP32Encoder encoder;
ToneESP32 buzzer(buzzerPin, 0); // Channel 0 for buzzer

// Variables not in Lab07.h
char sbuf[128];
char json_ExampleBuffer[256];
int currentState = STATE_IDLE;
int selectedNode = BLANK_7SEG_NUMBER;
int lastEncoderValue = 0;
unsigned long lastButtonPressTime = 0;
String lastReceivedNode = "";

unsigned long mscurrent = 0;
unsigned long mslastHeartbeat = 0;
unsigned long mslastMQTTCheck = 0;
unsigned long msLastEncoderCheck = 0;
unsigned long msLastButtonCheck = 0;

// Function prototypes
void connectWifi();
void setupMQTT();
void connectMQTT();
void sendHeartbeatMessage();
void registerForTopics();
void processMQTTMessage(char *topic, byte *payload, unsigned int);
void sendRegisteredForMessage();
void sendRingRingMessage(int destinationNode);
void displayDigit(int digit);
void playRingRingTone();
void BIST();
// ========== NEW DISPLAY TESTING FUNCTIONS ==========
void testShiftRegister() {
  Serial.println("Testing shift register pins...");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("Activating pin ");
    Serial.println(i);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, 1 << i);
    digitalWrite(latchPin, HIGH);
    delay(500);
  }
  // Clear register
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, 0);
  digitalWrite(latchPin, HIGH);
}

void testAllSegments() {
  Serial.println("Testing all display segments...");
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B11111111); // All segments ON
  digitalWrite(latchPin, HIGH);
  delay(2000);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, B00000000); // All segments OFF
  digitalWrite(latchPin, HIGH);
}

// ========== END NEW FUNCTIONS ==========

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }
  Serial.println("\n\nSystem starting...");

  // ========== ENHANCED DISPLAY INITIALIZATION ==========
  // Initialize shift register pins first
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  digitalWrite(latchPin, HIGH); // Default to HIGH
  
  // Run display hardware tests
  // testShiftRegister();
  // delay(1000);
  // testAllSegments();
  // delay(1000);
  // ========== END DISPLAY INITIALIZATION ==========

  // Initialize button with stronger debounce
  button.attach(pdButton, INPUT_PULLUP);
  button.interval(50); // Increased debounce time
  button.setPressedState(LOW);

  // Initialize encoder with single edge detection
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachSingleEdge(dtPin, clkPin); // More reliable than halfQuad
  encoder.clearCount();

  // Initialize buzzer with proper PWM setup
  buzzer.noTone();

  // Initialize built-in LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Run comprehensive self-test
  BIST();

  // Network setup
  connectWifi();
  setupMQTT();
  connectMQTT();


  // Visual startup confirmation
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, 0);
    delay(200);
    digitalWrite(LED_BUILTIN, 1);
    delay(150);
  }

  displayDigit(BLANK_7SEG_NUMBER);
  Serial.println("System ready");
}

void loop() {
  mscurrent = millis();

  psClient.loop();
  
  if ((mscurrent - mslastMQTTCheck) > MQTT_CONNECT_CHECK) {
    mslastMQTTCheck = mscurrent;
    MQTTConnected = psClient.connected();
    if (!MQTTConnected) connectMQTT();
  }

  button.update();

  if (((mscurrent - mslastHeartbeat) > HEARTBEAT_INTERVAL) &&
      HEARTBEAT_INTERVAL != 0 && MQTTConnected) {
    mslastHeartbeat = mscurrent;
    sendHeartbeatMessage();
  }

  switch (currentState) {
    case STATE_IDLE:
      if ((mscurrent - msLastEncoderCheck) > PB_UPDATE_TIME) {
        msLastEncoderCheck = mscurrent;
        long newValue = encoder.getCount();
        if (newValue != lastEncoderValue) {
          currentState = STATE_SELECT_DEST;
          // Determine direction of rotation
          int direction = (newValue > lastEncoderValue) ? 1 : -1;
          
          selectedNode = (selectedNode + direction) & 0x0F; 
          
          lastEncoderValue = newValue;
          displayDigit(selectedNode);
        }
      }
      break;


    case STATE_SELECT_DEST:
      if ((mscurrent - msLastEncoderCheck) > PB_UPDATE_TIME) {
        msLastEncoderCheck = mscurrent;
        long newValue = encoder.getCount();
        if (newValue != lastEncoderValue) {
          int direction = (newValue > lastEncoderValue) ? 1 : -1;
          selectedNode = (selectedNode + direction) & 0x0F; 
          lastEncoderValue = newValue;
          displayDigit(selectedNode);
        }
      }

      if (button.pressed()) {
        if (MQTTConnected) {
          sendRingRingMessage(selectedNode);
          digitalWrite(LED_BUILTIN, LOW);
          delay(100);
          digitalWrite(LED_BUILTIN, HIGH);
        }
        currentState = STATE_IDLE;
        displayDigit(BLANK_7SEG_NUMBER);
      }
      break;

    case STATE_RECEIVING:
      if ((mscurrent - msLastEncoderCheck) > PB_UPDATE_TIME) {
        msLastEncoderCheck = mscurrent;
        long newValue = encoder.getCount();
      if (newValue != lastEncoderValue) {
        int direction = (newValue > lastEncoderValue) ? 1 : -1;
        selectedNode = (selectedNode + direction) & 0x0F;
        lastEncoderValue = newValue;
        displayDigit(selectedNode);
      if (selectedNode != lastReceivedNode.substring(4).toInt()) {
        lastReceivedNode = "";
      }
    }
  }

  if (button.pressed()) {
    if (MQTTConnected) {
      sendRingRingMessage(selectedNode);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
    }
    currentState = STATE_IDLE;
    displayDigit(BLANK_7SEG_NUMBER);
  }
  break;
}
}


/**********************************************************
 * Helper functions - Provided by Professor
 *********************************************************/
void connectWifi() {
  // ProfNote:  Provided fully functional.
  // attempt to connect to the WiFi network
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print(" network");
  delay(10);
#ifdef LIPSCOMB
  WiFi.begin(ssid); // Lipscomb WiFi does NOT require a password
#elif defined(ETHERNET)
  WiFi.begin(ssid);
#else
  WiFi.begin(ssid, password); // For WiFi networks that DO require a password
#endif

  // advance a "dot-dot-dot" indicator until connected to WiFi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // report to console that WiFi is connected and print IP address
  Serial.print("MAC address = ");
  Serial.print(WiFi.macAddress());
  Serial.print(", connected as ");
  Serial.print(WiFi.localIP());
  Serial.println(".");
}

void setupMQTT() {
  // ProfNote:  Provided fully functional.
  // specify MQTT broker's domain name (or IP address) and port number
  Serial.print("Initalizing MQTT object with broker=");
  Serial.print(mqttBroker);
  Serial.print(" and port=");
  Serial.print(mqttPort);
  Serial.print("..");
  psClient.setServer(mqttBroker, mqttPort);

  // Specify callback function to process messages from the broker.
  psClient.setCallback(processMQTTMessage);
  Serial.println(".done");
}
void connectMQTT() {
  // ProfNote:  Provided fully functional.
  // Ping the server before trying to reconnect
  int WiFistatus = WiFi.status();
  if (WiFistatus != WL_CONNECTED) {
    Serial.println("WiFi check failed!  Trying to reconnect...");
    connectWifi();
  } else {
    // Serial.println("passed!");
    //  Try to connect to the MQTT broker (let loop() take care of retries)
    Serial.print("Connecting to MQTT with ClientID=");
    Serial.print(myClientID);
    Serial.print(" ... ");
    if (psClient.connect(myClientID)) {
      Serial.println("connected.");
      registerForTopics();
      sprintf(sbuf, "MQTT initialization complete\r\nReady!\r\n\r\n");
      Serial.print(sbuf);
    } else {
      // reconnect failed so print a console message, wait, and try again
      Serial.println(" failed!");
      Serial.print("MQTT client state=");
      Serial.println(psClient.state());
      Serial.print("(Is processor whitelisted?  ");
      Serial.print("MAC=");
      Serial.print(WiFi.macAddress());
      Serial.println(")");
    }
  }
}
/**********************************************************
 * ENDHelper functions - Provided by Professor
 *********************************************************/

void registerForTopics() {
  // register with MQTT broker for topics of interest to this node
  Serial.println("Registering for topics...");

  // Register for all topics in the topics array
  for (int i = 0; i < numTopics; i++) {
    psClient.subscribe(topics[i].c_str());
    Serial.print("Registered for: ");
    Serial.println(topics[i]);
  }

  Serial.println("Registration complete.");
}
void processMQTTMessage(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  // Null-terminate the payload to make it a valid C string
  payload[length] = '\0';

  // Convert to string for easier comparison
  String topicStr = String(topic);
  String payloadStr = String((char *)payload);

  Serial.print("Payload: ");
  Serial.println(payloadStr);

  // Handle topics message - request for topics this node is registered for
  if (topicStr.endsWith("/topics")) {
    Serial.println("Received topics request");
    sendRegisteredForMessage();
    return;
  }

  // Handle ringring message
  if (topicStr.endsWith("/ringring")) {
    // Parse JSON payload
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Extract source and destination nodes
    const char *srcNode = doc["srcNode"];
    const char *dstNode = doc["dstNode"];

    if (srcNode && dstNode) {
      Serial.print("Ring-Ring from ");
      Serial.print(srcNode);
      Serial.print(" to ");
      Serial.println(dstNode);

      if (String(dstNode) == nodeName) {
        lastReceivedNode = String(srcNode);
        currentState = STATE_RECEIVING;
        int senderID = lastReceivedNode.substring(4).toInt();

        encoder.clearCount();
        lastEncoderValue = senderID;
        encoder.setCount(senderID);
      
        displayDigit(senderID);
        playRingRingTone();
      }
    }
    return;
  }
}

void sendHeartbeatMessage() {
  JsonDocument doc;
  doc["NodeName"] = nodeName;
  doc["NodeType"] = nodeType;

  // Serialize JSON to buffer
  serializeJson(doc, json_ExampleBuffer);

  // Construct topic
  String heartbeatTopic = "ece/" + nodeName + "/heartbeat";

  // Publish message
  psClient.publish(heartbeatTopic.c_str(), json_ExampleBuffer);

  Serial.println("Sent heartbeat.");
}

void sendRegisteredForMessage() {
  // Create JSON payload
  JsonDocument doc;
  doc["NodeName"] = nodeName;

  // Create topic array using our predefined topics array
  JsonArray jsonTopics = doc["topics"].to<JsonArray>();
  for (int i = 0; i < numTopics; i++) {
    jsonTopics.add(topics[i]);
  }

  // Serialize JSON to buffer
  serializeJson(doc, json_ExampleBuffer);

  // Construct topic
  String registeredForTopic = "ece/" + nodeName + "/registeredFor";

  // Publish message
  psClient.publish(registeredForTopic.c_str(), json_ExampleBuffer);

  Serial.print("Sent registeredFor: ");
  Serial.println(json_ExampleBuffer);
}

void sendRingRingMessage(int destinationNode) {
  // Format destination node ID with leading zero if needed
  char destNodeID[3];
  snprintf(destNodeID, sizeof(destNodeID), "%02d", destinationNode);

  // Create destination node name
  String destNodeName = "node" + String(destNodeID);

  // Create JSON payload
  JsonDocument doc;
  doc["srcNode"] = nodeName;
  doc["dstNode"] = destNodeName;

  // Serialize JSON to buffer
  serializeJson(doc, json_ExampleBuffer);

  // Construct topic
  String ringringTopic = "ece/" + destNodeName + "/ringring";

  // Publish message
  psClient.publish(ringringTopic.c_str(), json_ExampleBuffer);

  Serial.print("Sent Ring-Ring to ");
  Serial.print(destNodeName);
  Serial.print(": ");
  Serial.println(json_ExampleBuffer);
}


void displayDigit(int digit) {
  if (digit < 0 || digit > BLANK_7SEG_NUMBER) digit = BLANK_7SEG_NUMBER;
  byte pattern = digitPatterns[digit];
  
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, pattern);
  digitalWrite(latchPin, HIGH);
  
  Serial.print("Display: ");
  Serial.println(pattern, DEC);
  Serial.println(pattern, BIN);
  Serial.println(digit < BLANK_7SEG_NUMBER ? String(digit, HEX) : "blank");
}


void playRingRingTone() {
  // First tone
  
  buzzer.tone(NOTE_C4, 250);
  delay(TONE_DURATION + 20); // Slightly longer to ensure completion
  
  // Pause
  buzzer.noTone();
  delay(PAUSE_DURATION);
  
  // Second tone
  buzzer.tone(NOTE_A4, TONE_DURATION);
  delay(TONE_DURATION + 20);
  
  // Cleanup
  buzzer.noTone();
}

void BIST() {
  Serial.println("\n=== Starting Built-In Self Test ===");

  // Comprehensive display test
  Serial.println("\n[TEST] 7-Segment Display:");
  Serial.println("Testing all digits 0-F and blank...");
  for (int i = 0; i <= BLANK_7SEG_NUMBER; i++) {
    displayDigit(i);
    delay(500);
  }

  // Buzzer test with verification
  Serial.println("\n[TEST] Buzzer:");
  Serial.println("Playing test tones - you should hear two distinct tones");
  playRingRingTone();
  Serial.println("Did you hear the tones? [Check hardware if not]");
  delay(1000);

  // Encoder test with more feedback
  Serial.println("\n[TEST] Rotary Encoder:");
  Serial.println("Rotate encoder in both directions - values should change");
  encoder.clearCount();
  long lastPos = 0;
  unsigned long startTime = millis();
  
  while (millis() - startTime < 10000) { // 10 second test
    long newPos = encoder.getCount();
    if (newPos != lastPos) {
      Serial.print("Encoder count: ");
      Serial.println(newPos);
      lastPos = newPos;
    }
    delay(50);
  }

  // Button test with visual feedback
  Serial.println("\n[TEST] Button:");
  Serial.println("Press the button - LED should blink when pressed");
  startTime = millis();
  bool pressed = false;
  
  while (millis() - startTime < 10000 && !pressed) {
    button.update();
    if (button.pressed()) {
      Serial.println("Button press detected!");
      pressed = true;
      digitalWrite(LED_BUILTIN, 0);
      delay(200);
      digitalWrite(LED_BUILTIN, 1);
    }
    delay(50);
  }

  if (!pressed) {
    Serial.println("No button press detected - check wiring");
  }

  Serial.println("\n=== BIST Complete ===");
  displayDigit(BLANK_7SEG_NUMBER);
  delay(1000);
}


