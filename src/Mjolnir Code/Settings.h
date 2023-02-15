#ifndef SETTINGS_H
#define SETTINGS_H

#include "Constants.h"


// OLED Display
SSD1306AsciiWire Oled;
#define OLED_ADDR 0x3C  // <----- OLED ADDRESS this may need changing, google it

//-------------------------------------------------------------------------------------------
// BIOS Deffinitions/pin assignments
//-------------------------------------------------------------------------------------------
#define PIN_PUSHER_RUN 5  //      Pusher       OP D5
#define PIN_ESC 10        //      ESC 1        OP D10     PB2     OC1B (PWM)
#define PIN_ESC_2 9       //      ESC 2        OP D9      PB1     OC1A (PWM)

#define PIN_ENCODER_CLK 2  //      Encoder CLK, IP D2      /INT0
#define PIN_ENCODER_DT 3   //      Encoder DT   IP D3      /INT1
#define PIN_ENCODER_SW 12  //      Encoder SW   IP D12     PB4     PCINT4
#define PIN_MAG_S 11       //      Mag Sensor   IP D11     PB3     PCINT3 Use NO and COM on Switch
#define PIN_TRIGGER 16     //      Trigger      IP D16/A2  PC2     PCINT10  Use NO and COM on switch.

#define PIN_SDA A4  //      OLED display IP A4/D18  I2C interface
#define PIN_SCL A5  //      OLED display IP A5/D19  I2C Interface

#define PIN_BATT_MON A7  //      Bat Volts    IP A7      ADC7 Analog battery reading, Resistor devideder 47K/10K

// Battery Controls - this needs to be adjusted for optomal perforance
byte BatteyType = BATTERY_3S;           // Default is 3S battery. this Variable gets changed via the config menu
int BatteryOffset = BATTERY_CALFACTOR;  // asign to a floating point veriable so can be adjusted in config menu

float BatteryCurrentVoltage = 99.0;
float BatteryMinVoltage = BATTERY_3S_MIN;
float BatteryMaxVoltage = BATTERY_3S_MAX;

// OLED Adjustments
int oled_adj = 2;
int oled_cols = 132;

#endif