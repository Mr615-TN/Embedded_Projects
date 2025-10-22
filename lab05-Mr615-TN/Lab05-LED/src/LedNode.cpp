/*******************************************************************************
 *     Program name: mqtt_ledNode.ino
 *          Version: 2.1
 *             Date: Oct 14, 2020
 *           Author: Greg Nordstrom
 *         Platform: NodeMCU 1.0 ESP8266.E12-based devkit board
 *    Additional HW: Active high LED w/ current-limiting resistor attached to D1
 * Additional files: mqtt_ledNode.h
 *  Req'd libraries: ESP8266WiFi, PubSubClient, ArduinoJson
 *
 * Modfied:  John Hutton
 * Date:     02/26/2023
 *
 * Turns on/off an LED connected to NodeMCU pin D1 in response to MQTT
 * "ledCommand" messages received from another node. Upon receiving such a
 * message, this node turns the LED on or off, and sends an MQTT "ledStatus"
 * message back to the commanding node verifying the new status of the LED.
 *
 * This program is intended to work with another MQTT node, mqtt_btnNodeXX,
 * but any node capable of sending a proper ledCommand message and receiving
 * an ledStatus message will work, including MQTT-Spy.
 *
 * All MQTT nodes must have a unique "clientID" in order to be distinguishable
 * by the MQTT broker. By default, this node's clientID is ledNodeXX, but you
 * will need to edit the #include'd "mqtt_ledNode.h" file and replace XX with
 * the two-digit ID number provided to you by your instructor.
 *
 * MQTT messages this node can receive:
 * ------------------------------------
 *    Topic: ledNodeXX/ledCommand
 *    Usage: Contains a command to turn the attached LED on or off. Note that
 *           the topic includes this node's Client ID (ledNodeXX). You must
 *           edit the mqtt_ledNode.h file to specify the actual ClientID
 *           value of this led node (i.e. change XX to your ID number). Also,
 *           the payload will contain the sender's Client ID (btnNodeXX). The
 *           actual Client ID of the sender is extracted from the message and
 *           used to send an ledStatus message back to the sender (see below).
 *  Payload: {"senderID":"btnNodeXX","cmd":"on" | "off"}
 *
 * MQTT messages this node can send:
 * ------------------------------------
 *    Topic: btnNodeXX/ledStatus
 *    Usage: Sends the LED's current status ("on" or "off") to the MQTT node
 *           that sent the ledCommand message (btnNodeXX). Note that this
 *           program extracts the sender's Client ID from the ledCommand message
 *           (see above) to ensure that only the sender gets the status message.
 *  Payload: {"ledStatus":"on" | "off", "msg":"some message text"}
 *
 * This program also displays status messages on a serial monitor (115200N81).
 *
 * As mentioned above, this node's functionality can be fully exercised using
 * an MQTT "sniffer" program such as MQTT-Spy (avaliable on GitHub). MQTT-Spy
 * is a Java program, which requires Java to be installed on the host machine.
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
#include <ArduinoJson.h>   // MQTT payloads are in JSON format
#include <Esp.h>           // Esp32 support
#include <PubSubClient.h>  // MQTT client
#include <WiFi.h>          // wi-fi support
#include "LedNode.h"  // this project's .h file

WiFiClient wfClient;              // create a wifi client
PubSubClient psClient(wfClient);  // create a pub-sub object (must be
                                  // associated with a wifi client)

// char buffer to store incoming/outgoing messages
char json_msgBuffer[200];

// buffer to store sprintf formatted strings for printing
char sbuf[80];

/* Prototype functions */
void connect_wifi();
void reconnect();
void register_myself();
void processMQTTMessage(char* topic, byte* json_payload, unsigned int length);
void sendLedStatusMessage(String senderID, String ledStatus,
                          String ledStatusMessage);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial connection
    delay(1);
  }
  Serial.println("Serial ready!");

  // setup Wifi connection
  Serial.print("\nSetting up network for IP => ");
  Serial.println(mqttBroker);
  connect_wifi();

  // initialize port pin and turn off LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, OFF);

  // specify MQTT broker's domain name (or IP address) and port number
  psClient.setServer(mqttBroker, mqttPort);

  // Specify callback function to process messages from broker
  psClient.setCallback(processMQTTMessage);

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
    digitalWrite(LED_BUILTIN, 0);  // active low
    delay(200);
    digitalWrite(LED_BUILTIN, 1);
    delay(150);
  }
  Serial.println("Network initialization complete");
}

void loop() {
  // This is largely a reactive program, and as such only uses
  // the main loop to maintain the MQTT broker connection and
  // periodically call the psClient.loop() code to ensure the
  // client stays active

  // reconnect to MQTT server if connection lost
  if (!psClient.connected()) {
    Serial.print("psClient.connected() returns => ");  // Debugging
    Serial.println(psClient.connected());              // Debugging
    Serial.println(
        "In Loop and appear to have lost connection...reconnecting");  // Debugging
    reconnect();
  }

  delay(1000);      // TBD changed from 10ms.
  psClient.loop();  // call periodically to keep client alive and well
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
  WiFi.begin(ssid);  // Lipscomb WiFi does NOT require a password
#elif defined(ETHERNET)
  WiFi.begin(ssid);
#else
  WiFi.begin(ssid, password);  // For WiFi networks that DO require a password
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

void processMQTTMessage(char* topic, byte* json_payload, unsigned int length) {
  // This code is called whenever a message previously registered for is
  // RECEIVED from the broker. Incoming messages are selected by topic,
  // then the payload is parsed and appropriate action taken. (NB: If only
  // a single message type has been registered for, then there's no need to
  // select on a given message type. In practice this may be rare though...)
  //
  // For info on sending MQTT messages inside the callback handler, see
  // https://github.com/bblanchon/ArduinoJson/wiki/Memory-model.
  //
  // NB: for guidance on creating proper jsonBuffer size for the json object
  // tree, see https://github.com/bblanchon/ArduinoJson/wiki/Memory-model

  // process messages by topic
  sprintf(sbuf, "%s/ledCommand", ledClientID);
  if (strcmp(topic, sbuf) == 0) {
    // received "ledCommand" message, so parse payload into an object tree
    // example payload: {"senderID":"btnNode14","cmd":"on"}

    // create a parse tree buffer and get a pointer to the buffer's root
    // NB: See https://github.com/bblanchon/ArduinoJson/wiki/Memory-model
    // for guidance on selecting proper jsonBuffer size
    JsonDocument jsonDoc;
    auto error = deserializeJson(jsonDoc, (char*)json_payload);
    Serial.println("Parse message packet is ...");
    serializeJsonPretty(jsonDoc, Serial);

    if (!error) {
      // JSON name-value pairs have been parsed into the parse tree, so
      // extract values associated with the names "senderID" and "cmd"
      // and store values in string variables
      String senderID = jsonDoc["senderID"];
      String cmd = jsonDoc["cmd"];
      Serial.println();
      Serial.print("cmd = ");
      Serial.println(cmd);

      // take action based on the command value
      if (cmd == cmdOn) {
        // turn the LED on
        digitalWrite(LED, ON);
        Serial.println("Turning LED ON.");

        // send an MQTT ledStatus message back to sending node
        sendLedStatusMessage(senderID, "on", "I've seen the light!");
      } else if (cmd == cmdOff) {
        // turn the LED off
        digitalWrite(LED, OFF);
        Serial.println("Turning LED OFF.");

        // send an MQTT ledStatus message back to sending node
        sendLedStatusMessage(senderID, "off",
                             "And darkness fell upon the land...");
      } else {
        // print console message that an unknown command value received
        Serial.print("Unknown command received (");
        Serial.print(cmd);
        Serial.println(")");
      }
    } else {
      // parse failed so print a console message and return to caller
      sprintf(sbuf, "failed to parse JSON payload (topic: %s)\r\n", topic);
      Serial.print(sbuf);
      return;
    }
  } else {
    // topic was registered with broker, but no processing code in place... :(
    sprintf(sbuf, "Topic: \"%s\" unhandled\r\n", topic);
    Serial.print(sbuf);
  }
}

void sendLedStatusMessage(String senderID, String ledStatus,
                          String ledStatusMessage) {
  // create a parse tree buffer and point to the root of the buffer
  JsonDocument root;

  // fill tree with message data
  root["ledStatus"] = ledStatus;
  root["msg"] = ledStatusMessage;

  // Send message
  serializeJson(root, json_msgBuffer);
  String msgTopic = senderID + "/ledStatus";
  const char* msg = msgTopic.c_str();  // put into char array form (zero copy!!)
  psClient.publish(msg, json_msgBuffer);
}

void register_myself() {
  // register with MQTT broker for topics of interest to this node
  Serial.print("Registering for topics...");
  sprintf(sbuf, "%s/ledCommand", ledClientID);
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
    Serial.print(ledClientID);
    Serial.print("...");
    if (psClient.connect(ledClientID.c_str())) {
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