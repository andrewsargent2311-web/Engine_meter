// =====================================================================
// ENGINE METER firmware -- nice!nano (nRF52840)
// Reads RPM (inductive pickup -> LMV331 comparator) + dual MAX31856
// thermocouple channels (EGT + CHT), and streams the data over BLE
// to the paraglide_vario_ESP32 flight computer using the Nordic UART
// Service (NUS) -- the exact same service/characteristic the ESP32's
// ble_manager.cpp already scans for and subscribes to.
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

// The shortest real interval is about 6.9 ms at 8,700 RPM. Below that,
// scale the lock-out to 40% of the last accepted period.
#define RPM_MIN_LOCKOUT_US    5000

// Capture raw interrupt edge intervals temporarily to diagnose pickup ringing.
#define RPM_DEBUG_EDGES       1

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
// RPM: measured from the period between pulses (captured in an ISR),
// not from a counted-pulses-per-window average -- this responds
// immediately to RPM changes instead of lagging behind a sample window.
// ---------------------------------------------------------------------
volatile uint32_t lastPulseUs = 0;
volatile uint32_t lastPeriodUs = 0;
volatile uint32_t lastPulseMillis = 0;
volatile uint32_t pulseCount = 0;      // debug: accepted pulses
volatile uint32_t rejectCount = 0;     // debug: rejected edges
volatile uint32_t edgeUs[32];
volatile uint8_t edgeN = 0;

void rpmISR() {
#if RPM_DEBUG_EDGES
  if (edgeN < 32) edgeUs[edgeN++] = micros();
#endif

  uint32_t now = micros();
  uint32_t dt = now - lastPulseUs;

  uint32_t lockout = (lastPeriodUs * 2) / 5;
  if (lockout < RPM_MIN_LOCKOUT_US) lockout = RPM_MIN_LOCKOUT_US;

  if (dt > lockout) {
    lastPeriodUs = dt;
    lastPulseUs = now;
    lastPulseMillis = millis();
    pulseCount++;
  } else {
    rejectCount++;
  }
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

// ---------------------------------------------------------------------
// OUTGOING PACKET FORMAT
// Fixed 10-byte layout, well under the default ~20 byte NUS notify MTU.
// This is a starting-point protocol -- change it freely, just keep the
// ESP32-side decoder (which you'll add in bleReadLatestEngineData()'s
// caller / drawParamotorPage()) in sync with whatever you settle on.
//
//   byte 0    : 0xAE  sync/header byte
//   byte 1-2  : uint16_t RPM, little-endian
//   byte 3-4  : int16_t  EGT, little-endian, in 0.1 degC units
//   byte 5-6  : int16_t  CHT, little-endian, in 0.1 degC units
//   byte 7    : status flags (bit0 = EGT fault, bit1 = CHT fault)
//   byte 8    : battery percentage (0-100)
//   byte 9    : checksum = XOR of bytes 0..8
// ---------------------------------------------------------------------
#define PKT_HEADER 0xAE

void sendEnginePacket(float rpm, float egtC, bool egtFault,
                       float chtC, bool chtFault, uint8_t batteryPercent) {
  uint8_t pkt[10];
  uint16_t rpmVal = (uint16_t)constrain(rpm, 0, 65535);
  int16_t egtVal = (int16_t)constrain(egtC * 10.0f, -3000, 30000);
  int16_t chtVal = (int16_t)constrain(chtC * 10.0f, -3000, 30000);

  pkt[0] = PKT_HEADER;
  pkt[1] = (uint8_t)(rpmVal & 0xFF);
  pkt[2] = (uint8_t)(rpmVal >> 8);
  pkt[3] = (uint8_t)(egtVal & 0xFF);
  pkt[4] = (uint8_t)(egtVal >> 8);
  pkt[5] = (uint8_t)(chtVal & 0xFF);
  pkt[6] = (uint8_t)(chtVal >> 8);
  pkt[7] = (egtFault ? 0x01 : 0x00) | (chtFault ? 0x02 : 0x00);
  pkt[8] = batteryPercent;

  uint8_t chk = 0;
  for (int i = 0; i < 9; i++) chk ^= pkt[i];
  pkt[9] = chk;

  if (Bluefruit.connected()) {
    bleuart.write(pkt, sizeof(pkt));
  }
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
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in units of 0.625ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);              // 0 = never stop advertising
}



void setup() {
  Serial.begin(115200);
  // Give the USB CDC monitor time to attach, but don't block normal use.
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 10000) {
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
  const uint32_t SEND_INTERVAL_MS = 200;  // 5 Hz telemetry

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

    float rpm = readRpm();

    float egtC = maxEGT.readThermocoupleTemperature();
    uint8_t egtFaultBits = maxEGT.readFault();
    bool egtFault = (egtFaultBits != 0);

    float chtC = maxCHT.readThermocoupleTemperature();
    uint8_t chtFaultBits = maxCHT.readFault();
    bool chtFault = (chtFaultBits != 0);

    uint8_t batteryPercent = readBatteryPercent();

    sendEnginePacket(rpm, egtC, egtFault, chtC, chtFault, batteryPercent);

    Serial.printf("RPM=%.0f  pulses=%lu  rejects=%lu  pin=%d  EGT=%.1fC%s  CHT=%.1fC%s  BAT=%u%%\n",
                  rpm, (unsigned long)pulseCount, (unsigned long)rejectCount,
                  digitalRead(PIN_RPM),
                  egtC, egtFault ? " [FAULT]" : "",
                  chtC, chtFault ? " [FAULT]" : "",
                  batteryPercent);
  }
}
