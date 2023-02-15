
bool BatteryFlat = false;

volatile bool ADCRead = false;
volatile unsigned long ADCValue = 0;
//--------------------------------------------------------------------------------------
ISR(ADC_vect)  // Battery Monitor - ADC ISR
{
  ADCRead = true;
  ADCValue = ADCL | (ADCH << 8);
}

