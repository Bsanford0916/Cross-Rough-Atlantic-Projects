#ifndef CONSTANTS_H
#define CONSTANTS_H

#define VERSION 20




#define BTN_LOW 0
#define BTN_HIGH 1
#define BTN_ROSE 2
#define BTN_FELL 3

#define BUTTON_PRESS_JUNK 3000
#define BUTTON_PRESS_LONG 1000
#define BUTTON_PRESS_SHORT 100
#define BTN_ACTION_NONE 0
#define BTN_ACTION_SHORT 1
#define BTN_ACTION_LONG 2

// System
#define SYSTEM_MODE_NORMAL 0
#define SYSTEM_MODE_MAGOUT 1
#define SYSTEM_MODE_CONFIG 2
#define SYSTEM_MODE_JAM 3
#define SYSTEM_MODE_LOWBATT 4

// Battery

#define BATTERY_3S 3
#define BATTERY_4S 4
#define BATTERY_3S_MIN 11.1    // 3S_Min volts = 9.6, thnk this should be 11.1
#define BATTERY_3S_MAX 13.0    // 3S_Max volts = 13.2
#define BATTERY_4S_MIN 13.2    // 4S_Min volts = 13.2 and this 14.8
#define BATTERY_4S_MAX 16.8    // 4S_Max volts = 16.8
#define BATTERY_CALFACTOR 0.0  // Default Battery Offset Adjustment for calibration

// EEPROM addresses -- critical alignment here
#define ADDR_MSF 0             // 0 1 byte
#define ADDR_ROFA 1            // 1
#define ADDR_ROFB 2            // 2
#define ADDR_BURST 3           // 3
#define ADDR_MAGSIZE 4         // 4
#define ADDR_ACCEL 5           // 5/6      2 byte
#define ADDR_DECEL 7           // 7/8      2 byte
#define ADDR_STARTD 9          // 9/10     2 byte
#define ADDR_STOPD 11          // 11/12    2 byte
#define ADDR_PULSE_HIGH 13     // 13/14    2 byte
#define ADDR_PULSE_LOW 15      // 15/16    2 byte
#define ADDR_PULSE_RETRACT 17  // 17/18    2 byte
#define ADDR_BTN_LS 19         // 19       not used, referance code
#define ADDR_BTN_LL 20         // 20       not used, referance code
#define ADDR_BTN_CS 21         // 21       not used, referance code
#define ADDR_BTN_CL 22         // 22       not used, referance code
#define ADDR_BTN_RS 23         // 23       not used, referance code
#define ADDR_BTN_RL 24         // 24       not used, referance code
#define ADDR_BTN_ROT 25        // 25       not used, referance code
#define ADDR_BAT_TYPE 26       // 26
#define ADDR_BAT_OFFSET 27     // 27/28/29    2 byte
#define ADDR_PROA_BASE 0       // 30
#define ADDR_PROB_BASE 31      // 31
#define ADDR_SECTORL 29        // 30

#endif