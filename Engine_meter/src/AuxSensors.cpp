#include "AuxSensors.h"

namespace {

constexpr uint8_t PIN_VBAT = 4;  // P0.04 / AIN2 on the nice!nano
constexpr float VBAT_DIV_RATIO = 0.713f;
constexpr uint32_t BATTERY_SAMPLE_INTERVAL_MS = 5000;
constexpr uint8_t ADC_SAMPLE_COUNT = 16;
constexpr float BATTERY_FILTER_ALPHA = 0.25f;

struct BatteryPoint {
  float voltage;
  uint8_t percent;
};

constexpr BatteryPoint BATTERY_CURVE[] = {
    {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.90f, 60}, {3.80f, 40},
    {3.75f, 30},  {3.70f, 20}, {3.60f, 10}, {3.50f, 5},  {3.30f, 0},
};
constexpr size_t BATTERY_CURVE_SIZE =
    sizeof(BATTERY_CURVE) / sizeof(BATTERY_CURVE[0]);

uint32_t lastBatterySampleMillis = 0;
float filteredBatteryVolts = 0.0f;
uint8_t batteryPercent = 0;
bool hasBatterySample = false;

float readBatteryVolts() {
  uint32_t adcTotal = 0;
  for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; ++i) {
    adcTotal += analogRead(PIN_VBAT);
    delay(2);
  }

  const float averageAdc = adcTotal / static_cast<float>(ADC_SAMPLE_COUNT);
  const float adcVolts = averageAdc * (3600.0f / 4096.0f) / 1000.0f;
  return adcVolts / VBAT_DIV_RATIO;
}

uint8_t voltageToPercent(float voltage) {
  if (voltage >= BATTERY_CURVE[0].voltage) {
    return 100;
  }

  for (size_t i = 1; i < BATTERY_CURVE_SIZE; ++i) {
    if (voltage >= BATTERY_CURVE[i].voltage) {
      const BatteryPoint& upper = BATTERY_CURVE[i - 1];
      const BatteryPoint& lower = BATTERY_CURVE[i];
      const float fraction =
          (voltage - lower.voltage) / (upper.voltage - lower.voltage);
      const float percent =
          lower.percent + fraction * (upper.percent - lower.percent);
      return static_cast<uint8_t>(percent + 0.5f);
    }
  }

  return 0;
}

}  // namespace

void beginAuxSensors() {
  analogReference(AR_INTERNAL);
  analogReadResolution(12);
  lastBatterySampleMillis = millis() - BATTERY_SAMPLE_INTERVAL_MS;
}

uint8_t readBatteryPercent() {
  const uint32_t now = millis();
  if (!hasBatterySample ||
      now - lastBatterySampleMillis >= BATTERY_SAMPLE_INTERVAL_MS) {
    lastBatterySampleMillis = now;
    const float measuredVolts = readBatteryVolts();

    if (hasBatterySample) {
      filteredBatteryVolts +=
          BATTERY_FILTER_ALPHA * (measuredVolts - filteredBatteryVolts);
    } else {
      filteredBatteryVolts = measuredVolts;
      hasBatterySample = true;
    }

    batteryPercent = voltageToPercent(filteredBatteryVolts);
  }

  return batteryPercent;
}
