#include "AuxSensors.h"

namespace {

constexpr uint32_t BATTERY_SAMPLE_INTERVAL_MS = 5000;
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
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
  NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;

  NRF_SAADC->CH[0].CONFIG =
      (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos) |
      (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
      (SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos) |
      (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
      (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
      (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
      (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos);

  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDDHDIV5;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;

  for (int i = 1; i < 8; ++i) {
    NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
  }

  volatile int16_t adcBuffer = 0;
  NRF_SAADC->RESULT.PTR = reinterpret_cast<uint32_t>(&adcBuffer);
  NRF_SAADC->RESULT.MAXCNT = 1;

  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
  while (!NRF_SAADC->EVENTS_CALIBRATEDONE) {
  }

  int32_t adcTotal = 0;
  int validSamples = 0;
  for (int i = 0; i < 10; ++i) {
    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_START = 1;
    while (!NRF_SAADC->EVENTS_STARTED) {
    }

    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (!NRF_SAADC->EVENTS_END) {
    }

    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->TASKS_STOP = 1;
    while (!NRF_SAADC->EVENTS_STOPPED) {
    }

    if (adcBuffer > 0) {
      adcTotal += adcBuffer;
      ++validSamples;
    }
  }

  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;

  if (validSamples == 0) {
    return 0.0f;
  }

  const float averageAdc = adcTotal / static_cast<float>(validSamples);
  return averageAdc * 18.0f / 4096.0f;
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
