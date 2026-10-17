// =====================================================================
// ENGINE METER firmware -- nice!nano (nRF52840)
// Reads RPM (inductive pickup -> LMV331 comparator) + dual MAX31856
// thermocouple channels (EGT + CHT), and streams NMEA telemetry over BLE
// using the Nordic UART Service (NUS) to Gaggle and the ESP32 vario.
// EMS carries engine readings; PEMBAT carries battery percentage.
// EMS fields after the identifier: pressure, fuel flow, fuel remaining,
// fuel used, ambient temperature, CHT, EGT, water temperature, RPM.
// Unused fields use 999999 (pressure), 9999 (fuel), or 99 (temperatures).
// CHT/EGT are clamped to -300..1500 C and rounded degrees C x100 (99 means
// unavailable); RPM is x100 (9999 means unavailable, and a computed 9999
// is sent as 10000). RPM is a gated 5 s average with optional 1 s fast
// attack on real RPM changes, rounded to 10 RPM with 7 RPM hysteresis.
// RPM_TIMEOUT_MS reports zero immediately after a stop/stall. $PEMBAT,
// <percent>*HH carries 0-100% battery and is controlled by SEND_PEMBAT.
// Checksums are XOR as two uppercase hex digits; sentences end in CRLF.
//
// Board: nice!nano (nRF52840), Adafruit nRF52 Arduino core
// Build system: PlatformIO (VS Code) -- see platformio.ini in this folder.
// Libraries: Adafruit MAX31856 is pulled in via lib_deps in platformio.ini.
//   Bluefruit (bluefruit.h) ships with the Adafruit nRF52 framework already.
//
// Pin mapping traced from Engine_monitor_V1.kicad_sch. Adafruit's nRF52
// core numbers digital pins as: P0.xx -> pin xx, P1.xx -> pin (32+xx).
//
// =====================================================================

#include <Arduino.h>   // required in PlatformIO (.ino files get this added automatically)
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <bluefruit.h>
#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include "AuxSensors.h"

// ---------------------------------------------------------------------
// PINS (from schematic global labels)
// ---------------------------------------------------------------------
#define PIN_SDO     32   // P1.00 -- MAX31856 SDO  -> MCU MISO (shared bus)
#define PIN_CS_CHT  11   // P0.11 -- MAX31856 #2 (CHT) chip select
#define PIN_CS_EGT  36   // P1.04 -- MAX31856 #1 (EGT) chip select
#define PIN_SCK     47   // P1.15 -- shared SPI clock
#define PIN_SDI     45   // P1.13 -- MCU MOSI -> MAX31856 SDI (shared bus)
#define PIN_RPM     43   // P1.11 -- comparator output (LMV331), pulled up by R9

// ---------------------------------------------------------------------
// RPM CONFIG
// ---------------------------------------------------------------------
// How many spark pulses per crank revolution the pickup sees.
// Most single-cylinder 2-stroke paramotor engines: 1 pulse/rev.
// Change to 0.5 for a twin sharing one kill lead, 2 for a wasted-spark
// 4-stroke twin that fires every pulse but only every 2nd is a power
// stroke, etc. -- tune this to what you actually observe.
#define PULSES_PER_REV        1.0f

// If no pulse has arrived in this long, report RPM = 0 (engine off/stalled)
// rather than showing a stale reading.
#define RPM_TIMEOUT_MS         1500

#define RPM_WINDOW_MS          5000   // averaging window
#define RPM_SAMPLE_MS          200    // snapshot interval (= SEND_INTERVAL_MS)
#define RPM_ROUND_TO           10     // round displayed RPM to this step
#define RPM_HYST               7.0f   // hysteresis in RPM for display rounding
#define RPM_FAST_ATTACK        1      // 1 = shrink window on real RPM change
#define RPM_FAST_WINDOW_MS     1000
#define RPM_FAST_THRESHOLD     0.10f  // 10% deviation triggers fast window
#define RPM_HIST_LEN           (RPM_WINDOW_MS / RPM_SAMPLE_MS + 1)

// The shortest real interval is about 6.9 ms at 8,700 RPM. Below that,
// scale the lock-out to 40% of the last accepted period, but only when
// that period is below 100 ms (> 600 RPM).
#define RPM_MIN_LOCKOUT_US    5000
#define RPM_MAX_ADAPT_US      100000

// Capture raw interrupt edge intervals temporarily to diagnose pickup ringing.
#define RPM_DEBUG_EDGES       0

// ---------------------------------------------------------------------
// BLE -- Nordic UART Service, matching ble_manager.cpp on the ESP32 side
// ---------------------------------------------------------------------
BLEUart bleuart;  // Bluefruit's BLEUart *is* the Nordic UART Service --
                   // same service UUID (6E400001-...) and TX characteristic
                   // (6E400003-...) the ESP32 vario already looks for.

// ---------------------------------------------------------------------
// Thermocouple amps -- software SPI (bit-banged) so we can use the
// arbitrary GPIOs the board routes them on, sharing one SCK/SDI/SDO bus
// with separate chip-selects.
// ---------------------------------------------------------------------
Adafruit_MAX31856 maxEGT(PIN_CS_EGT, PIN_SDI, PIN_SDO, PIN_SCK);
Adafruit_MAX31856 maxCHT(PIN_CS_CHT, PIN_SDI, PIN_SDO, PIN_SCK);

// ---------------------------------------------------------------------
// RPM: raw pulse periods are captured in the ISR; accepted-pulse snapshots
// provide a gated average, with a short fast-attack window for RPM changes.
// ---------------------------------------------------------------------
volatile uint32_t lastPulseUs = 0;
volatile uint32_t lastPeriodUs = 0;
volatile uint32_t lastPulseMillis = 0;
volatile uint32_t pulseCount = 0;      // accepted-pulse count for RPM averaging and diagnostics
volatile uint32_t rejectCount = 0;     // debug: rejected edges
volatile uint32_t edgeUs[32];
volatile uint8_t edgeN = 0;

struct RpmSnap {
  uint32_t count;
  uint32_t us;
};

static RpmSnap rpmHist[RPM_HIST_LEN];
static uint8_t rpmHistHead = 0;  // next write index
static uint8_t rpmHistN = 0;     // valid entries
static float displayedRpm = 0.0f;

void rpmISR() {
#if RPM_DEBUG_EDGES
  if (edgeN < 32) edgeUs[edgeN++] = micros();
#endif

  uint32_t now = micros();
  uint32_t dt = now - lastPulseUs;

  uint32_t lockout = RPM_MIN_LOCKOUT_US;
  if (lastPeriodUs != 0 && lastPeriodUs < RPM_MAX_ADAPT_US) {
    uint32_t adaptive = (lastPeriodUs * 2) / 5;
    if (adaptive > lockout) lockout = adaptive;
  }

  if (dt <= lockout) {
    rejectCount++;
    return;
  }

  // A gap longer than the timeout means the engine was stopped, so this is
  // the first pulse of a new run and there is no valid period yet.
  bool firstPulse = (dt > (uint32_t)RPM_TIMEOUT_MS * 1000UL);
  lastPeriodUs = firstPulse ? 0 : dt;
  lastPulseUs = now;
  lastPulseMillis = millis();
  pulseCount++;
}

float readRpm() {
  noInterrupts();
  uint32_t period = lastPeriodUs;
  uint32_t lastMs = lastPulseMillis;
  interrupts();

  if (lastMs == 0 || (millis() - lastMs) > RPM_TIMEOUT_MS || period == 0) {
    return 0.0f;
  }
  return (1000000.0f / period) / PULSES_PER_REV * 60.0f;
}

float computeWindowedRpm(uint32_t windowMs) {
  if (rpmHistN < 2) {
    return 0.0f;
  }

  const uint8_t newestIndex =
      static_cast<uint8_t>((rpmHistHead + RPM_HIST_LEN - 1) % RPM_HIST_LEN);
  const uint32_t requestedBack = windowMs / RPM_SAMPLE_MS;
  const uint8_t availableBack = static_cast<uint8_t>(rpmHistN - 1);
  const uint8_t back = requestedBack < availableBack
                           ? static_cast<uint8_t>(requestedBack)
                           : availableBack;
  const uint8_t oldestIndex =
      static_cast<uint8_t>((newestIndex + RPM_HIST_LEN - back) % RPM_HIST_LEN);

  const uint32_t dCount = rpmHist[newestIndex].count - rpmHist[oldestIndex].count;
  const uint32_t dUs = rpmHist[newestIndex].us - rpmHist[oldestIndex].us;
  if (dCount == 0 || dUs == 0) {
    return 0.0f;
  }

  return (dCount * 1000000.0f / dUs) / PULSES_PER_REV * 60.0f;
}

float getSmoothedRpm() {
  static uint32_t lastSampleMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastSampleMs >= RPM_SAMPLE_MS) {
    lastSampleMs = nowMs;
    RpmSnap snap;
    noInterrupts();
    snap.count = pulseCount;
    snap.us = lastPulseUs;
    interrupts();

    rpmHist[rpmHistHead] = snap;
    rpmHistHead = static_cast<uint8_t>((rpmHistHead + 1) % RPM_HIST_LEN);
    if (rpmHistN < RPM_HIST_LEN) {
      rpmHistN++;
    }
  }

  if (readRpm() == 0.0f) {
    rpmHistN = 0;
    rpmHistHead = 0;
    displayedRpm = 0.0f;
    return 0.0f;
  }

  const float avgRpm = computeWindowedRpm(RPM_WINDOW_MS);
  float smoothedRpm = avgRpm;
#if RPM_FAST_ATTACK == 1
  const float fastRpm = computeWindowedRpm(RPM_FAST_WINDOW_MS);
  if (avgRpm > 0.0f &&
      fabsf(fastRpm - avgRpm) / avgRpm > RPM_FAST_THRESHOLD) {
    smoothedRpm = fastRpm;
  }
#endif

  const float candidate =
      roundf(smoothedRpm / RPM_ROUND_TO) * RPM_ROUND_TO;
  if (fabsf(smoothedRpm - displayedRpm) >= RPM_HYST ||
      displayedRpm == 0.0f || smoothedRpm == 0.0f) {
    displayedRpm = candidate;
  }
  return displayedRpm;
}

#define SEND_PEMBAT 1

void sendNmeaSentence(char* sentence, size_t capacity, int length) {
  if (length <= 0 || static_cast<size_t>(length) >= capacity) {
    return;
  }

  char* checksumMarker = strchr(sentence, '*');
  if (checksumMarker == nullptr || checksumMarker - sentence < 1 ||
      checksumMarker[1] == '\0' || checksumMarker[2] == '\0') {
    return;
  }

  uint8_t checksum = 0;
  for (const char* cursor = sentence + 1; cursor < checksumMarker; ++cursor) {
    checksum ^= static_cast<uint8_t>(*cursor);
  }

  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  checksumMarker[1] = HEX_DIGITS[(checksum >> 4) & 0x0F];
  checksumMarker[2] = HEX_DIGITS[checksum & 0x0F];

  if (Bluefruit.connected()) {
    bleuart.write(reinterpret_cast<const uint8_t*>(sentence), length);
  }
}

// Example outputs (checksum is XOR of bytes between '$' and '*'):
// rpm=3100, CHT=180.0, EGT=550.0, no faults:
// $EMS,999999,9999,9999,9999,99,18000,55000,99,310000*7C
// CHT ok, EGT fault, rpm=3100:
// $EMS,999999,9999,9999,9999,99,18000,99,99,310000*4C
// Both faulted, rpm=0:
// $EMS,999999,9999,9999,9999,99,99,99,99,0*47
// Battery 85%: $PEMBAT,85*2E
void sendEngineNmea(float rpm, float egtC, bool egtFault, float chtC,
                    bool chtFault, uint8_t batteryPercent) {
  const long chtValue =
      chtFault || !isfinite(chtC)
          ? 99
          : lroundf(constrain(chtC, -300.0f, 1500.0f) * 100.0f);
  const long egtValue =
      egtFault || !isfinite(egtC)
          ? 99
          : lroundf(constrain(egtC, -300.0f, 1500.0f) * 100.0f);
  long rpmValue = isfinite(rpm) && rpm > 0.0f ? lroundf(rpm * 100.0f) : 0;
  if (rpmValue == 9999) {
    rpmValue = 10000;
  }

  char sentence[96];
  int length = snprintf(sentence, sizeof(sentence),
                        "$EMS,999999,9999,9999,9999,99,%ld,%ld,99,%ld*00\r\n",
                        chtValue, egtValue, rpmValue);
  sendNmeaSentence(sentence, sizeof(sentence), length);

#if SEND_PEMBAT
  length = snprintf(sentence, sizeof(sentence), "$PEMBAT,%u*00\r\n",
                    static_cast<unsigned int>(batteryPercent));
  sendNmeaSentence(sentence, sizeof(sentence), length);
#endif
}

// ---------------------------------------------------------------------
// BLE connect/disconnect callbacks -- just for a status LED / Serial log
// ---------------------------------------------------------------------
void connectCallback(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  char name[32] = {0};
  conn->getPeerName(name, sizeof(name));
  Serial.print("[BLE] Connected to ");
  Serial.println(name);
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  Serial.println("[BLE] Disconnected -- advertising again");
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addName();  // only room for a shortened name ("ParaE") after flags+txpower+128-bit UUID

  // Full name goes in the scan response so the vario's scanner shows
  // "ParaEngineMeter" instead of the truncated version.
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in units of 0.625ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);              // 0 = never stop advertising
}



void setup() {
  Serial.begin(115200);
  // Give the USB CDC monitor time to attach, but don't block normal use.
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 1500) {
    delay(10);
  }
  Serial.println("[BOOT] USB serial ready; initializing engine meter");

  beginAuxSensors();

  // ---- RPM input ----
  pinMode(PIN_RPM, INPUT);  // external 10k pull-up (R9) already on board
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmISR, RISING);

  // ---- Thermocouples ----
  maxEGT.begin();
  maxEGT.setThermocoupleType(MAX31856_TCTYPE_K);
  maxCHT.begin();
  maxCHT.setThermocoupleType(MAX31856_TCTYPE_K);

  // ---- BLE ----
  Bluefruit.configPrphConn(247, BLE_GAP_EVENT_LENGTH_DEFAULT, 8, 8);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("ParaEngineMeter");  // shows up in the vario's BT scan list
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  bleuart.begin();

  startAdv();
  Serial.println("[BOOT] Engine meter ready, advertising as ParaEngineMeter");
}

void loop() {
  static uint32_t lastSend = 0;
  const uint32_t SEND_INTERVAL_MS = RPM_SAMPLE_MS;  // 5 Hz telemetry

#if RPM_DEBUG_EDGES
  if (edgeN >= 32) {
    uint32_t t[32];
    noInterrupts();
    memcpy(t, (const void*)edgeUs, sizeof(t));
    edgeN = 0;
    interrupts();
    for (int i = 1; i < 32; i++) Serial.printf("%lu ", (unsigned long)(t[i] - t[i-1]));
    Serial.println();
  }
#endif

  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();

    float rpm = getSmoothedRpm();
    float rawRpm = readRpm();

    float egtC = maxEGT.readThermocoupleTemperature();
    uint8_t egtFaultBits = maxEGT.readFault();
    bool egtFault = (egtFaultBits != 0);

    float chtC = maxCHT.readThermocoupleTemperature();
    uint8_t chtFaultBits = maxCHT.readFault();
    bool chtFault = (chtFaultBits != 0);

    uint8_t batteryPercent = readBatteryPercent();

    sendEngineNmea(rpm, egtC, egtFault, chtC, chtFault, batteryPercent);

    Serial.printf("RPM=%.0f raw=%.0f  pulses=%lu  rejects=%lu  pin=%d  EGT=%.1fC%s  CHT=%.1fC%s  BAT=%u%%\n",
                  rpm, rawRpm, (unsigned long)pulseCount, (unsigned long)rejectCount,
                  digitalRead(PIN_RPM),
                  egtC, egtFault ? " [FAULT]" : "",
                  chtC, chtFault ? " [FAULT]" : "",
                  batteryPercent);
  }
}
