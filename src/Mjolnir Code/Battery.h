#ifndef BATTERY_H
#define BATTERY_H

#include "Constants.h"
bool BatteryFlat = false;

volatile bool ADCRead = false;
volatile unsigned long ADCValue = 0;
//--------------------------------------------------------------------------------------
ISR(ADC_vect)  // Battery Monitor - ADC ISR
{
  ADCRead = true;
  ADCValue = ADCL | (ADCH << 8);
}

//------------------------------------ Keep tabs on the battery ----------------------------------------
void ProcessBatteryMonitor() {

#define BM_STATUS_IDLE 0
#define BM_STATUS_READING 1
#define BM_STATUS_READY 2

  static byte CurrentStatus = 0;
  static unsigned long LastCheck = 0;

  // Every 500ms, start a background ADC read
  if (CurrentStatus == BM_STATUS_IDLE) {
    if (millis() - LastCheck < 500) {
      return;
    }

    LastCheck = millis();
    CurrentStatus = BM_STATUS_READING;
    ADCSRA |= (1 << ADSC);  // Run the ADC
    return;
  }

  // When the ADC has finished it's conversion, proceed to the processing step
  if (CurrentStatus == BM_STATUS_READING) {
    if (!ADCRead) {
      return;  // Nothing to see here
    }
    ADCRead = false;
    CurrentStatus = BM_STATUS_READY;
  }


  if (CurrentStatus != BM_STATUS_READY)
    return;

#define NUM_SAMPLES 6
  static byte CollectedSamples = 0;
  static float SampleAverage = 0;
  int BatteryOffsetfloat = 0;

  if (CollectedSamples < NUM_SAMPLES) {
    CollectedSamples++;
    SampleAverage += (float)ADCValue;
  } else {
    float BatOffsetFloat = BatteryOffset * 0.1;
    BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0) / 1024.0 * (float)((47.0 + 10.0) / 10.0)) + BatOffsetFloat;  // Voltage dividor - 47k and 10k
    if (BatteryCurrentVoltage < BatteryMinVoltage) {
      if (BatteryCurrentVoltage > 1.6)  // If the current voltage is 0, we are probably debugging
      {
        BatteryFlat = true;
      } else {
        BatteryFlat = false;
      }
    } else {
      BatteryFlat = false;
    }
    CollectedSamples = 0;
    SampleAverage = 0;
  }

  // Reset back to the default position.
  LastCheck = millis();
  CurrentStatus = BM_STATUS_IDLE;
}

#endif

