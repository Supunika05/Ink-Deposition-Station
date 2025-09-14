#pragma once # Prevent duplicate declarations
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <WString.h> 
#include "secrets.h"
#include "AWS_connect.h"

extern const char* TOPIC_CONTROL;
extern String rx;
extern WiFiClientSecure net;
extern MQTTClient client;

void handleLine(const String& line);
void messageHandler(String &topic, String &payload);
void connectAWS();
