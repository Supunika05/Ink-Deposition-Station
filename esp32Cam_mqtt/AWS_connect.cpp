#include <sys/_intsup.h>
#include "WString.h"
#include "HardwareSerial.h"
#include "esp32-hal-gpio.h"
#include "secrets.h"
#include "board_config.h"
#include "esp_camera.h"
#include "AWS_connect.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>

const char* TOPIC_CONTROL = "laptop/control/camera";

WiFiClientSecure net;
MQTTClient client;

extern volatile bool cameraActive = false;      // currently allowed to capture/publish
extern volatile bool requestStop = false;       // set by MQTT handler to ask loop to deinit safely
extern volatile bool publishing = false;        // set by loop while publishing

volatile bool ledState = false;

void messageHandler(String &topic, String &payload) {
  // Allocate a JSON document
  StaticJsonDocument<200> doc;

  // Parse JSON payload
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("command")) {
    String msg = doc["command"];
    msg.trim();
    Serial.print("Command: ");
    Serial.println(msg);

    if (msg.equalsIgnoreCase("ON")) {
      cameraActive = true;

      esp_err_t err = esp_camera_init(&config);
      if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        cameraActive = false;
        // DO NOT block here. Return so MQTT loop can continue.
        return;
      } else {
        Serial.println("Camera activated");
      }
    }
    else if (msg.equalsIgnoreCase("OFF")) {
      Serial.println("OFF received - requesting camera stop");
      // stop capturing new frames immediately
      cameraActive = false;
      // request loop to deinit camera safely when it is not publishing
      requestStop = true;
      esp_camera_deinit(); 
    } 
    else if (msg.equalsIgnoreCase("FLASH")) {
      ledState = !ledState;
      if (ledState == true) {
        digitalWrite(LED_GPIO_NUM, HIGH);
      } 
      else {
        digitalWrite(LED_GPIO_NUM, LOW);
      }
    }
    else {
      Serial.println("Invalid command");
    }
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