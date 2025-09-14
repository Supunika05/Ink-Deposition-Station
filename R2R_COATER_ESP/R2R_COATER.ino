#include <Arduino.h>
#include "driver/ledc.h"
#include <math.h>
#include <cmath>
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <EEPROM.h>
#include "secrets.h"
#include "WiFi.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "wifi_connect.h"
#include "AWS_connect.h"

//
// Roll-to-Roll Controller — ESP32-S3
// - 4 injectors (syringe pumps):
//     • process uses flow (uL/min)
//     • manual load uses fixed RPM (5 RPM)
// - Drive roll:
//     • process uses user m/min (Forward only)
//     • jog uses fixed 1.0 m/min preset
// - Microstepping: x16 (TMC2209 MS1=HIGH, MS2=HIGH)
// - Smooth, non-blocking ramps (50 Hz) + DIR settle hold
//
// Commands (newline-terminated):
//   INK01_SPEED <ml/min> ... INK04_SPEED <ml/min>
//   INK01 FORWARD|BACKWARD|STOP ... INK04 ...
//   DRIVE_SPEED <m_per_min>
//   DRIVE FORWARD|BACKWARD|STOP      (jog preset; ignored during process)
//   PROCESS START
//   PROCESS STOP
//

//////////////////// USER SETTINGS ////////////////////
#define MICROSTEP_ALL            16
#define STEPS_PER_REV            200

// Drive roll mechanics
#define DRIVE_DIAMETER_M         0.030f     // 30 mm
#define DRIVE_GEAR_RATIO         1.0f
#define DRIVE_JOG_M_PER_MIN      1.0f       // jog preset
#define DRIVE_MAX_M_PER_MIN      10.0f

// Injector mechanics (10 mL syringe)
#define SYRINGE_ID_MM            14.5f
#define LEADSCREW_PITCH_MM       2.0f
#define INJECTOR_LOAD_RPM        5.0f
#define INJECTOR_MAX_uL_PER_MIN  20000.0f

// EN polarity (most TMC2209 boards are EN=LOW to enable)
#define ENABLE_ACTIVE_LOW        1

// ====== GPIOs ======
#define DRV_STEP  2
#define DRV_DIR   42
#define DRV_EN    1

#define INK1_STEP 47
#define INK1_DIR  48
#define INK1_EN   21

#define INK2_STEP 5
#define INK2_DIR  4
#define INK2_EN   6

#define INK3_STEP 8
#define INK3_DIR  7
#define INK3_EN   3

#define INK4_STEP 0
#define INK4_DIR  45
#define INK4_EN   35
// ====================================

// LEDC
#define CH_DRV   0
#define CH_INK1  1
#define CH_INK2  2
#define CH_INK3  3
#define CH_INK4  4
#define LEDC_RES_BITS 8  // duty 0..255
////////////////////////////////////////////////////////

static const float PI_F = 3.14159265358979f;

// Derived constants
static const float DRIVE_CIRC_M = PI_F * DRIVE_DIAMETER_M;
static const float DRIVE_HZ_PER_M_PER_MIN =
  (STEPS_PER_REV * MICROSTEP_ALL * DRIVE_GEAR_RATIO) / (60.0f * DRIVE_CIRC_M); // ~565.885 Hz per (m/min)

static const float SYR_AREA_MM2 = PI_F * (SYRINGE_ID_MM * SYRINGE_ID_MM) * 0.25f;
static const float MM_PER_uL    = 1.0f / SYR_AREA_MM2;          // plunger travel per uL
static const float REV_PER_uL   = MM_PER_uL / LEADSCREW_PITCH_MM;  // motor rev per uL
static const float INK_HZ_PER_uL_PER_MIN =
  (REV_PER_uL * STEPS_PER_REV * MICROSTEP_ALL) / 60.0f;            // ~161.49 Hz per (uL/min)
static const float INK_LOAD_HZ =
  (INJECTOR_LOAD_RPM * STEPS_PER_REV * MICROSTEP_ALL) / 60.0f;     // 5 RPM → ~266.67 Hz

// Direction + settle
enum class Dir { FWD, BWD };
static const uint32_t DIR_SETTLE_MS = 3;    // hold pulses off this long after flipping DIR

struct Motor {
  const char* name;
  int pin_step, pin_dir, pin_en, ch;

  volatile bool  running;
  volatile Dir   dir;
  volatile float targetHz;
  volatile float currentHz;
  bool           processControlled;

  // after changing DIR, wait until this time before pulsing again
  volatile uint32_t dirHoldUntilMs;

  // new: allow per-motor inversion to handle driver wiring/polarity
  bool dirInverted;

  Motor(const char* n, int step, int dirPin, int enPin, int ledcCh)
  : name(n),
    pin_step(step), pin_dir(dirPin), pin_en(enPin), ch(ledcCh),
    running(false), dir(Dir::FWD),
    targetHz(0.0f), currentHz(0.0f),
    processControlled(false), dirHoldUntilMs(0),
    dirInverted(false) {}
};

// LEDC helpers
inline void pulseStop(const Motor& m) { ledcWrite(m.ch, 0); }
inline void pulseStart(const Motor& m, float hz) {
  float f = hz; if (f < 1.0f) f = 1.0f;
  ledcWriteTone(m.ch, f);
  ledcWrite(m.ch, 128); // 50% duty
}
inline void setDir(const Motor& m, Dir d) {
  bool writeHigh = (d == Dir::FWD);
  if (m.dirInverted) writeHigh = !writeHigh;
  digitalWrite(m.pin_dir, writeHigh ? HIGH : LOW);
  // debug print (optional, good for checking)
  Serial.printf("SETDIR %s -> %s (written %s) on pin %d\n", m.name,
                (d==Dir::FWD) ? "FWD" : "BWD",
                writeHigh ? "HIGH" : "LOW",
                m.pin_dir);
}
inline void setEn(const Motor& m, bool en) {
  if (m.pin_en < 0) return;
  pinMode(m.pin_en, OUTPUT);
#if ENABLE_ACTIVE_LOW
  digitalWrite(m.pin_en, en ? LOW : HIGH);
#else
  digitalWrite(m.pin_en, en ? HIGH : LOW);
#endif
}

// Motors
Motor M_drv ("DRIVE", DRV_STEP, DRV_DIR, DRV_EN, CH_DRV);
Motor M_ink1("INK01", INK1_STEP, INK1_DIR, INK1_EN, CH_INK1);
Motor M_ink2("INK02", INK2_STEP, INK2_DIR, INK2_EN, CH_INK2);
Motor M_ink3("INK03", INK3_STEP, INK3_DIR, INK3_EN, CH_INK3);
Motor M_ink4("INK04", INK4_STEP, INK4_DIR, INK4_EN, CH_INK4);

Motor* MOTORS[] = { &M_drv, &M_ink1, &M_ink2, &M_ink3, &M_ink4 };
const int NMOT = sizeof(MOTORS)/sizeof(MOTORS[0]);

// Setpoints from GUI
static volatile float drv_process_m_per_min = 0.0f;
static volatile float ink_ul_per_min[4]     = {0,0,0,0};
static volatile bool  processRunning        = false;

// Find motor by name
Motor* findMotor(const String& s) {
  for (int i=0;i<NMOT;i++) if (s.equalsIgnoreCase(MOTORS[i]->name)) return MOTORS[i];
  return nullptr;
}

// Run/stop with DIR settle
void runMotor(Motor& m, Dir d, float targetHz, bool markProcessOwner) {
  Serial.printf("RUN %s -> %s hz=%.2f processOwner=%d\n", m.name,
                (d==Dir::FWD) ? "FWD" : "BWD", targetHz, markProcessOwner ? 1 : 0);
  const bool dirChanged = (m.dir != d);
  m.dir = d; setDir(m, d);

  if (dirChanged) {
    // stop pulses and hold a few ms so DIR is stable before next STEP
    pulseStop(m);
    m.dirHoldUntilMs = millis() + DIR_SETTLE_MS;
  }

  m.targetHz = targetHz;
  m.running  = (targetHz > 0.0f);
  m.processControlled = markProcessOwner;
}

void stopMotor(Motor& m) {
  m.running = false;
  m.processControlled = false;
  pulseStop(m);
}

// Parser
String rx;
void handleLine(const String& line) {
  String s = line; s.trim(); if (!s.length()) return;
  int sp = s.indexOf(' ');
  String a = (sp<0)? s : s.substring(0,sp);
  String b = (sp<0)? "" : s.substring(sp+1); b.trim();

  // DRIVE_SPEED <m/min>
  if (a.equalsIgnoreCase("DRIVE_SPEED")) {
    float v = b.toFloat();
    if (!isfinite(v) || v < 0) v = 0;
    if (v > DRIVE_MAX_M_PER_MIN) v = DRIVE_MAX_M_PER_MIN;
    drv_process_m_per_min = v;
    return;
  }
  // DRIVE FORWARD/BACKWARD/STOP (jog; ignore while process is running)
  if (a.equalsIgnoreCase("DRIVE")) {
    if (processRunning) return;
    b.toUpperCase();
    if (b=="FORWARD")      runMotor(M_drv, Dir::FWD, DRIVE_JOG_M_PER_MIN * DRIVE_HZ_PER_M_PER_MIN, false);
    else if (b=="BACKWARD")runMotor(M_drv, Dir::BWD, DRIVE_JOG_M_PER_MIN * DRIVE_HZ_PER_M_PER_MIN, false);
    else if (b=="STOP")    stopMotor(M_drv);
    return;
  }
  // INKxx_SPEED <ml/min>
  if (a.startsWith("INK") && a.endsWith("_SPEED")) {
    int us = a.indexOf('_'); String name = a.substring(0,us);
    Motor* m = findMotor(name); if (!m) return;
    int idx = name.substring(3).toInt() - 1; if (idx<0 || idx>3) return;
    float q = b.toFloat(); if (!isfinite(q) || q<0) q = 0;
    if (q > INJECTOR_MAX_uL_PER_MIN) q = INJECTOR_MAX_uL_PER_MIN;
    ink_ul_per_min[idx] = q;
    return;
  }
  // INKxx FORWARD|BACKWARD|STOP (manual load at fixed RPM)
  if (a.startsWith("INK")) {
    Motor* m = findMotor(a); if (!m) return;
    b.toUpperCase();
    if (b=="FORWARD")       runMotor(*m, Dir::FWD, INK_LOAD_HZ, false);
    else if (b=="BACKWARD") runMotor(*m, Dir::BWD, INK_LOAD_HZ, false);
    else if (b=="STOP")     stopMotor(*m);
    return;
  }
    // INVERT INK02  (toggle direction inversion for testing)
  if (a.equalsIgnoreCase("INVERT")) {
    Motor* m = findMotor(b);
    if (!m) { Serial.printf("INVERT: unknown '%s'\n", b.c_str()); return; }
    m->dirInverted = !m->dirInverted;
    Serial.printf("%s dirInverted=%d\n", m->name, m->dirInverted ? 1 : 0);
    return;
  }

  // PROCESS START/STOP
  if (a.equalsIgnoreCase("PROCESS")) {
    b.toUpperCase();
    if (b=="START") {
      // Drive (Forward only)
      const float drvHz = drv_process_m_per_min * DRIVE_HZ_PER_M_PER_MIN;
      if (drvHz > 0.0f) runMotor(M_drv, Dir::FWD, drvHz, true);
      else              stopMotor(M_drv);

      // Injectors with Q>0 (Forward only)
      Motor* inks[4] = {&M_ink1,&M_ink2,&M_ink3,&M_ink4};
      for (int i=0;i<4;i++) {
        const float q = ink_ul_per_min[i];
        if (q > 0.0f)  runMotor(*inks[i], Dir::FWD, q * INK_HZ_PER_uL_PER_MIN, true);
        else           stopMotor(*inks[i]); // also cancels any prior manual load
      }
      processRunning = true;
    } else if (b=="STOP") {
      for (int i=0;i<NMOT;i++) stopMotor(*MOTORS[i]);
      processRunning = false;
    }
    return;
  }
}

// Setup/loop
unsigned long lastRamp = 0;

void setup() {
  Serial.begin(115200);

  // Pins & LEDC
  for (int i=0;i<NMOT;i++) {
    Motor& m = *MOTORS[i];
    pinMode(m.pin_dir, OUTPUT);
    pinMode(m.pin_step, OUTPUT);
    setDir(m, m.dir);
    setEn(m, true);

    ledcSetup(m.ch, 100.0, LEDC_RES_BITS); // initial freq
    ledcAttachPin(m.pin_step, m.ch);
    pulseStop(m);
  }

  // WIFI/AWS connection
  connectWiFi();
  connectAWS();
  client.subscribe(TOPIC_CONTROL);

  Serial.println("R2R controller ready");
}

void loop() {
  // WIFI/AWS
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
  }
  // Read lines
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r') {
      if (rx.length()) { handleLine(rx); rx=""; }
    } else {
      rx += c;
      if (rx.length() > 160) rx.remove(0, rx.length());
    }
  }

  // 50 Hz ramp loop
  const unsigned long now = millis();
  if (now - lastRamp >= 20) {
    lastRamp = now;
    const float RAMP_HZ = 250.0f; // Hz per 20ms tick

    for (int i=0;i<NMOT;i++) {
      Motor& m = *MOTORS[i];

      // ramp toward target
      const float tgt = (m.running ? m.targetHz : 0.0f);
      if (fabsf(tgt - m.currentHz) <= RAMP_HZ) m.currentHz = tgt;
      else if (tgt > m.currentHz)             m.currentHz += RAMP_HZ;
      else                                    m.currentHz -= RAMP_HZ;

      // respect DIR settle hold
      if (m.running && m.currentHz > 1.0f) {
        if (millis() >= m.dirHoldUntilMs) pulseStart(m, m.currentHz);
        else                              pulseStop(m);
      } else {
        pulseStop(m);
      }
    }
  }
}
