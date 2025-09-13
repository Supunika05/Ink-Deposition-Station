#pragma once # Prevent duplicate declarations
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "AWS_connect.h"
#include "board_config.h"

extern const char* TOPIC_CONTROL;

extern camera_config_t config;

extern WiFiClientSecure net;
extern MQTTClient client;

extern volatile bool cameraActive;      // currently allowed to capture/publish
extern volatile bool requestStop;       // set by MQTT handler to ask loop to deinit safely
extern volatile bool publishing;        // set by loop while publishing

extern volatile bool ledState;

void messageHandler(String &topic, String &payload);
void connectAWS();