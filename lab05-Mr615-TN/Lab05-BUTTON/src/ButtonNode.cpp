/*******************************************************************************
 *     Program name: mqtt_btnNode_starter_code.ino
 *          Version: 2.0
 *             Date: Aug 24, 2019
 *         Modified: May 22, 2023
 *           Author: John Hutton
 *         Platform: Adafruit Huzza32 Feather ESP32 board.
 *    Additional HW: Two n/o pushbuttons connected to D5 ("On") and D6 ("Off")
 * Additional files: mqtt_btnNode_starter_code.h
 *  Req'd libraries: WiFi, PubSubClient, ArduinoJson, Bounce2
 *
 * Changes:
 *   5/22/23:  Updated for ESP32 board.
 *
 * Sends/receives MQTT messages to/from a remote LED node (ledNodeXX), turning
 * its LED on or off. Use mqtt_ledcontroller_starter_code.h to set this button
 * node's Client ID, as well as the remote LED node's Client ID and the MQTT
 * broker's connection information.
 *
 * This node sends and receives the following MQTT messages, where btnNodeXX
 * is this node's Client ID and ledNodeXX is the remote LED node's Client ID:
 *
 *   Sends:
 *      Topic: "ledNodeXX/ledCommand"
 *      Usage: To instruct ledNodeXX to turn on/off its LED
 *    Payload: {"senderID":"btnNodeXX","cmd":"on" | "off"}
 *
 *   Receives:
 *      Topic: "btnNodeXX/ledStatus"
 *      Usage: Reports the current status of ledNodeXX's LED
 *    Payload: {"ledStatus":"on" | "off", "msg":"some message text"}
 *
 * This program also displays status messages on a serial terminal (115200N81)
 * along with the message text contained in a received ledStatus message.
 *
 * Finally, note that this node's functionality can be fully exercised using an
 * MQTT message "sniffer" such as MQTT-Spy (available on GitHub). MQTT-Spy is a
 * Java program, which requires Java to be installed on the host machine.
 *
 * DEVELOPER NOTES:
 * This program uses the public domain PubSubClient library to perform MQTT
 * messaging functions, and all message payloads are encoded in JSON format.
 *  1. Default message size, including header, is only 128 bytes (rather small).
 *  2. Increase/decrease by changing MQTT_MAX_PACKET_SIZE inside PubSubClient.h.
 *  3. Recommended size is 512 bytes.
 *
 ******************************************************************************/

// included configuration file and support libraries
#include "ButtonNode.h"   // this project's .h file
#include <ArduinoJson.h>  // MQTT payloads are in JSON format
#include <Bounce2.h>      // pushbutton debouncer library
#include <Esp.h>          // Esp32 support
#include <PubSubClient.h> // MQTT client
#include <WiFi.h>         // wi-fi support


WiFiClient wfClient;             // create a wifi client
PubSubClient psClient(wfClient); // create a pub-sub object (must be
                                 // associated with a wifi client)

// char buffer to store incoming/outgoing messages
char json_msgBuffer[200];

// buffer to store sprintf formatted strings for printing
char sbuf[80];

// Button debouncing variables
// ProjH suggests using Bounce2 for your buttons, but also
// good to leverage something else you have working.
Bounce2::Button OnButton = Bounce2::Button();
Bounce2::Button OffButton = Bounce2::Button();

// prototype functions
void connect_wifi();
void processMQTTMessage_B(char *topic, byte *json_payload, unsigned int length);
void register_myself();
void reconnect();
void sendLedCommand(String command);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial connection
    delay(1);
  }
  Serial.println("Serial ready!");

  // Initialize buttons
  OnButton.attach(PB_ON, INPUT_PULLUP); // Initialize the ON button with internal pull-up
  OnButton.interval(DEBOUNCE_INTERVAL); // Set debounce interval
  OnButton.setPressedState(LOW);        // Button is pressed when input is LOW (active low)
  
  OffButton.attach(PB_OFF, INPUT_PULLUP); // Initialize the OFF button with internal pull-up
  OffButton.interval(DEBOUNCE_INTERVAL);  // Set debounce interval
  OffButton.setPressedState(LOW);         // Button is pressed when input is LOW (active low)

  // setup Wifi connection
  Serial.print("\nSetting up network for IP => ");
  Serial.println(mqttBroker);
  connect_wifi();

  // specify MQTT broker's domain name (or IP address) and port number
  psClient.setServer(mqttBroker, mqttPort);

  // Specify callback function to process messages from broker
  psClient.setCallback(processMQTTMessage_B);

  // connect to MQTT broker
  if (!psClient.connected()) {
    Serial.println(
        "In Setup and appear to have lost connection...reconnecting");
    reconnect();
  }

  // finally, flash the on-board LED five times to let user know
  // that the NodeMCU board has been initialized and ready to go
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, 1); // active high
    delay(200);
    digitalWrite(LED_BUILTIN, 0);
    delay(150);
  }
  
  Serial.println("Network initialization complete");
  Serial.println("Ready to send commands to LED node");
}

void loop() {
  // This is largely a reactive program, and as such only uses
  // the main loop to maintain the MQTT broker connection and
  // periodically call the psClient.loop() code to ensure the
  // client stays active

  // reconnect to MQTT server if connection lost
  if (!psClient.connected()) {
    Serial.print("psClient.connected() returns => "); // Debugging
    Serial.println(psClient.connected());             // Debugging
    Serial.println(
        "In Loop and appear to have lost connection...reconnecting"); // Debugging
    reconnect();
  }

  // Update the state of the buttons
  OnButton.update();
  OffButton.update();
  
  // Check if ON button was pressed
  if (OnButton.pressed()) {
    Serial.println("ON button pressed");
    sendLedCommand(cmdOn);
  }
  
  // Check if OFF button was pressed  
  if (OffButton.pressed()) {
    Serial.println("OFF button pressed");
    sendLedCommand(cmdOff);
  }

  psClient.loop(); // call periodically to keep client alive and well

  delay(10); // May need to tweak this delay, may not be needed
}

/**********************************************************
 * Helper functions
 *********************************************************/
void connect_wifi() {
  // in an attempt to remove the annoying garbled text on
  // startup, print a couple of blank lines (with delay)
  Serial.println();
  delay(100);
  Serial.println();
  delay(100);

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
    delay(300);
    Serial.print(".");
  }

  // report to console that WiFi is connected and print IP address
  Serial.print("MAC address = ");
  Serial.print(WiFi.macAddress());
  Serial.print(", connected as ");
  Serial.println(WiFi.localIP());
}

// Function to send a command to the LED node
void sendLedCommand(String command) {
  // Create a JSON document
  JsonDocument root;
  
  // Add the sender ID and command to the JSON document
  root["senderID"] = buttonClientID;
  root["cmd"] = command;
  
  // Serialize the JSON document to a buffer
  serializeJson(root, json_msgBuffer);
  
  // Create the topic string
  String topic = ledClientID + "/ledCommand";
  
  // Publish the message
  psClient.publish(topic.c_str(), json_msgBuffer);
  
  Serial.print("Sent command: ");
  Serial.print(command);
  Serial.print(" to ");
  Serial.println(topic);
}

void processMQTTMessage_B(char *topic, byte *json_payload,
                          unsigned int length) {
  // This code is called whenever a message previously registered for is
  // RECEIVED from the broker. Incoming messages are selected by topic,
  // then the payload is parsed and appropriate action taken.

  // Create a buffer for the expected topic
  char expectedTopic[50];
  sprintf(expectedTopic, "%s/ledStatus", buttonClientID.c_str());

  if (strcmp(topic, expectedTopic) == 0) {
    // Create a JSON document
    JsonDocument jsonDoc;
    
    // Deserialize the JSON payload
    auto error = deserializeJson(jsonDoc, (char *)json_payload);

    // If deserialization was successful
    if (!error) {
      // Extract the LED status and message
      String ledStatus = jsonDoc["ledStatus"].as<String>();
      String msg = jsonDoc["msg"].as<String>();
      
      // Print the LED status and message
      Serial.println("\nReceived LED Status Message:");
      Serial.print("LED Status: ");
      Serial.println(ledStatus);
      Serial.print("Message: ");
      Serial.println(msg);
      Serial.println();
    } else {
      // If JSON parsing fails, print an error message
      Serial.println("Failed to parse JSON payload");
    }
  } else {
    // If the topic doesn't match, print an unhandled topic message
    sprintf(sbuf, "Topic: \"%s\" unhandled\r\n", topic);
    Serial.print(sbuf);
  }
}

void register_myself() {
  // register with MQTT broker for topics of interest to this node
  Serial.print("Registering for topics...");
  sprintf(sbuf, "%s/ledStatus", buttonClientID.c_str());
  psClient.subscribe(sbuf);

  Serial.println(" done");
}

void reconnect() {
  // Loop until the pub-sub client connects to the MQTT broker
  while (!psClient.connected()) {
    // attempt to connect to MQTT broker
    Serial.print("Connecting to MQTT broker (");
    Serial.print(mqttBroker);
    Serial.print(") as ");
    Serial.print(buttonClientID);
    Serial.print("...");
    if (psClient.connect(buttonClientID.c_str())) {
      Serial.println(" connected");
      // clientID MUST BE UNIQUE for all connected clients
      // can also include username, password if broker requires it
      // (e.g. psClient.connect(clientID, username, password)

      // once connected, register for topics of interest
      register_myself();
      sprintf(sbuf, "MQTT initialization complete\r\nReady!\r\n\r\n");
      Serial.print(sbuf);
    } else {
      // reconnect failed so print a console message, wait, and try again
      Serial.println(" failed.");
      Serial.println("Trying again in 5 sec. (Is processor whitelisted?)");
      // wait 5 seconds before retrying
      delay(5000);
    }
  }
}
