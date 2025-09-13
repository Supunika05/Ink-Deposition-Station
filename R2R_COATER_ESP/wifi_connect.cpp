#include <Arduino.h>
#include <EEPROM.h>
#include "wifi_connect.h"
 
WiFiManager wm;
 
bool shouldSaveConfig = false;
 
// Buffer for Wi-Fi credentials
char WIFI_SSID[32];
char WIFI_PASSWORD[32];
 
// Callback to save config after web server updates SSID and password
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
 
// Save SSID and password to EEPROM
void saveCredentials(const char* newSSID, const char* newPass) {
  Serial.println("Saving WiFi credentials to EEPROM...");
  
  // Save SSID
  for (int i = 0; i < 32; i++) {
    EEPROM.write(i, (i < strlen(newSSID)) ? newSSID[i] : 0);
  }
  for (int i = 0; i < 32; i++) {
    EEPROM.write(100 + i, (i < strlen(newPass)) ? newPass[i] : 0);
  }
  EEPROM.commit();
}
 
// Read SSID and password from EEPROM
void readCredentials() {
  Serial.println("Reading WiFi credentials from EEPROM...");
  
  for (int i = 0; i < 32; i++) {
    WIFI_SSID[i] = EEPROM.read(0 + i);
  }
  WIFI_SSID[31] = '\0';
 
  for (int i = 0; i < 32; i++) {
    WIFI_PASSWORD[i] = EEPROM.read(100 + i);
  }
  WIFI_PASSWORD[31] = '\0';
 
  Serial.println("SSID: ");
  Serial.println(WIFI_SSID);
  
  delay(500);
}

void connectWiFi() {
  Serial.println("Connecting Wi-Fi...");
  WiFi.mode(WIFI_STA);
  // Set WiFiManager save config callback
  EEPROM.begin(512);
  wm.setSaveConfigCallback(saveConfigCallback);

  if (!wm.autoConnect("ESP32_Config")) {
    Serial.println("Failed to connect via WiFiManager");
    readCredentials();  // Fallback to EEPROM if WiFiManager fails
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  // Save config if needed
  if (shouldSaveConfig) {
    saveCredentials(wm.getWiFiSSID().c_str(), wm.getWiFiPass().c_str());
    Serial.println("Credentials saved.");
    ESP.restart();  // Restart to apply settings
  }

  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nWi-Fi OK, IP: %s\n", WiFi.localIP().toString().c_str());
}