#include <sys/_intsup.h>
#include "WString.h"
#include "HardwareSerial.h"
#include "esp32-hal-gpio.h"
#include "secrets.h"
#include "esp_camera.h"
#include "AWS_connect.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>

const char* TOPIC_CONTROL = "laptop/control/command";
extern String rx;

WiFiClientSecure net;
MQTTClient client;


void messageHandler(String &topic, String &payload) {
  // Parse JSON payload for a "command" field and call handleLine() with it
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("command")) {
    String msg = doc["command"].as<String>();
    msg.trim();
    Serial.print("Command (from MQTT): ");
    Serial.println(msg);

    // call the command parser directly with the payload command
    handleLine(msg);
  }
}


void connectAWS() {
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.begin(AWS_IOT_ENDPOINT, 8883, net);
  client.setCleanSession(true);
  Serial.print("Connecting to AWS IoT");
  while (!client.connect(THINGNAME)) { Serial.print("."); delay(200); }
  Serial.println("\nAWS IoT OK");

  client.onMessage(messageHandler);
  client.subscribe(TOPIC_CONTROL);
}
