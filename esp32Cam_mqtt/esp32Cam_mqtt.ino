#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <EEPROM.h>
#include "WiFi.h"
#include "esp_camera.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "board_config.h"
#include "wifi_connect.h"
#include "AWS_connect.h"
#include "camera_pins.h"

#define TOPIC_STREAM "esp32_cam/stream"
#define TOPIC_CONTROL "esp32_cam/control"

const int mqttBuf = 1024 * 24; // 24 KB
camera_config_t config;

#define EEPROM_SIZE 1  // only need 1 byte for cameraActive flag

// ---------- Helpers ----------
void saveCameraState(bool state) {
  EEPROM.write(0, state ? 1 : 0);
  EEPROM.commit();
}

bool readCameraState() {
  return EEPROM.read(0) == 1;
}

void logResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("Reset reason: %d (SW_CPU_RESET=12)\n", (int)r);
  Serial.printf("Free heap: %u | Free PSRAM: %u\n", ESP.getFreeHeap(), ESP.getFreePsram());
}

// Capture -> copy to PSRAM -> return fb
uint8_t* capture_to_psram(size_t &outLen) {
  outLen = 0;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("Capture failed (fb==NULL)"); return nullptr; }

  uint8_t *buf = (uint8_t*)ps_malloc(fb->len);
  if (!buf) {
    Serial.printf("ps_malloc(%u) failed\n", fb->len);
    esp_camera_fb_return(fb);
    return nullptr;
  }

  memcpy(buf, fb->buf, fb->len);
  outLen = fb->len;

  esp_camera_fb_return(fb); // Free DMA/FB quickly
  return buf;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println("\nBoot");
  logResetReason();

  EEPROM.begin(EEPROM_SIZE);

  if (!psramFound()) {
    Serial.println("WARNING: PSRAM not found.");
  }

  // Camera config
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 15;
  config.fb_count = 1;

#if defined(CAMERA_GRAB_WHEN_EMPTY)
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
#endif

  connectWiFi();
  connectAWS();

  client.onMessage(messageHandler);
  client.subscribe(TOPIC_CONTROL);  // Subscribe to control topic

  // Restore camera state from EEPROM
  cameraActive = readCameraState();
  if (cameraActive) {
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) Serial.println("Camera resumed streaming after reboot/reconnect");
  }

  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
}

// ---------- Main Loop ----------
void loop() {
  client.loop();
  yield();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi drop, reconnecting...");
    WiFi.reconnect();
    delay(300);
    return;
  }
  if (!client.connected()) {
    Serial.println("MQTT drop, reconnecting...");
    connectAWS();
    client.onMessage(messageHandler);
    client.subscribe(TOPIC_CONTROL);

    // Restart streaming if it was previously ON
    if (readCameraState() && !cameraActive) {
      esp_err_t err = esp_camera_init(&config);
      if (err == ESP_OK) {
        cameraActive = true;
        Serial.println("Camera resumed streaming after MQTT reconnect");
      }
    }
  }

  if (!cameraActive) {
    delay(100);
    return;
  }

  size_t imgLen = 0;
  uint8_t *img = capture_to_psram(imgLen);
  if (!img) {
    client.loop();
    delay(150);
    return;
  }

  if (imgLen > (size_t)mqttBuf) {
    free(img);
    client.loop();
    delay(150);
    return;
  }

  unsigned long t0 = millis();
  publishing = true;
  bool ok = client.publish(TOPIC_STREAM, (const char*)img, imgLen);
  client.loop();
  publishing = false;

  unsigned long dt = millis() - t0;
  Serial.printf("Publish %s in %lums | Free heap:%u\n", ok ? "OK" : "FAIL", dt, ESP.getFreeHeap());

  free(img);
  delay(180);
}
