#pragma once # Prevent duplicate declarations
#include <WiFiManager.h>

extern WiFiManager wm;
extern bool shouldSaveConfig;
extern char WIFI_SSID[32];
extern char WIFI_PASSWORD[32];

void saveConfigCallback();
void saveCredentials(const char* newSSID, const char* newPass);
void readCredentials();
void connectWiFi();