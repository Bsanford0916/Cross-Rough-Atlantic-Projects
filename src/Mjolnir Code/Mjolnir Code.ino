/* 11-2-23 v20
 * For the Nerf Comunity, open progect...
 * 
 * Brushless | Solenoid Pusher | Singal Trigger | I2C OLED 128 x 64 display | Rotary Encoder with PB_Switch | Serial DeBUG 57600Buad
 *  
 * ----------------------
 * HARDWARE CONFIGURTION
 * ----------------------
 *  
 *   D5  Output - Solenoide Pusher, Mosfet Trigger
 *   D10 Output - Flywheels ESC stage 1 signal
 *   D9  Output - Flywheels ESC stage 2 signal (not developed yet)
 *   
 *   D2  Input - Encoder CLK   
 *   D3  Input - Encoder DT 
 *   D12 Input - Encoder SW     
 *   
 *   D11 Input - Mag Sensor  On the switch use NO to D11 and COM to GND.
 *   A2  Input - Trigger     On the switch use NO to A2/D16 and COM to GND.
 *
 *   A4  SDA - I2C interface to OLED display. A4/D18
 *   A5  SCL - I2C Interface to OLED display. A5/D19
 *   
 *   A7  Analog battery reading, Resistor devideder 47K/10K.   <====== VERY IMPORTANT, else code wont start
 *   
 *   
 *   Requires 
 *    - SSD1306 128 x 64 OLED Display NOTE: OLED_ADDR may need to be set correctly
 *    - Rotary Encoder with inbuilt PB Switch. DT to middle pin, CLK to oustide, GND to outside
 *    - Battery Resistor Deviderr on A7 47k/10K, with a bat, else code will say low bat
 *  
 *   
 * ------------------------------------------------------------------------------------------------------------
 * INSTRUCTIONS
 * Copy the three files:
 * 
 *    Thors_Hammer_Menus_v19.ino
 *    Logo.h
 *    MenuHelpers.h
 *  
 * Into one folder, and initiate the Adrouno IDE by clicking Thors_Hammer_menus_v16.ino
 * Under Tools set the Board Type:Arduino Nano, Processor:ATmega328P (old Bootloader), and Com port
 * 
 * Verify program, top left.
 * It may error, requiring libarie files. These can be found under Tools/Manager Libaries..
 * 
 * Once it Verifies OK...
 * Upload to Arduino ( the next button)
 * If updoad fails try changinging the Processor under Tools/Processos, to ATmega328P
 * 
 * Set Serial Monitor, to 57600 baud, for debug output. Not required.
 * 
 * First time upload will load default values into EEprom and configuration screen
 * 
 * ROTARY ENCODER
 * Use Rotary Encoder Push Button (PB), Short click to change Select Fire Mode. Mode it is on default display (Single / Burst: 2 / Full Auto)
 * Use a Long Click to get into the Configuration screen, short click to get out when on the EXIT tag, top or bottom of menu
 * 
 * OLED DISPLAY - Default home Screen
 * Bat volts: 11.4v
 * Select fire mode: Single, Burst: 2, Full Auto (Use short clicks to change Select Fire Mode
 * ROF: MAX, Rate Of Fire, default is max Pusher speed, ie changeing value, via config screen will slow it down
 * Pwr: 50%, Flywheels output, default is on 50%, you can change the power in the config menu, it will go faster or slower
 * Pro: A, Deafualt Profile we are useing, does not change
 * Ammo Counter: reset via mag switch. no trap on zero count.
 * Use short clicks to change Select Fire Mode (Single / Burst: 2 / Full Auto)
 * Long Click to get into the Configuration screen, short click to get out when on the EXIT tag, top or bottom of menu
 * 
 * OLED DISPLAY - Configuration Screen. Long Click to enter. 
 * To change values, Click to get # rotate to change value, click to accept. Values are saved to EEPROM instantly
 * 
 * EXIT           // Exit config back to default run screen
 * Power:         // Flywheel Power 30 - 100% Default: %50
 * A ROF          // Full Auto, Rate Of Fire 0(Max) - 150 Default: Max
 * B ROF          // Burst, Rate Of Fire 0(Max) - 150 Default: Max
 * Burst          // Burst Rate (number of darts to fire) 2-99 Default: 2
 * MagS           // Mag Size capacity 0-99 Default: 18
 * Ramp U         // Fly Wheel Start Ramp Up time 0 - 5000 Default: 0 (Fastest)
 * Ramp D         // Fly Wheel Stop Ramp Down time 0 - 5000 Default: 4000
 * Dwel U         // Ramp up Dwell time 0 - 5000 Default: 0 Use this to hold the rev
 * Dwel D         // Ramp down Dwell time 0 - 5000 Default: 200 Use this to hold the rev
 * SP Hi          // Solenoid Pulse High Time 0 - 1000 Default: 35 
 * SP Low         // Solenoid Pulse Low Time 0 - 1000 Default: 45
 * SP Ret         // Solenoid Pulse Retract Time 0- 1000 Default: 45
 * Bat Type       // 3S or 4S default 3S
 * Bat Off        // Battery Offset for calibration, increments of 0.1 displayed as interger
 * EXIT           // Exit config back to default run screen
 * 
 * To load factory default settings in EEPROM, Hold trigger ON during power up or reset, let go when screen says too.
 * 
 * ------------------------------------------------------------------------
 * SOLENOID TUNING
 * ------------------------------------------------------------------------
 *    In order to get a good solenoid tune, you need to have 3 things: 
 *     - Push-out Time while LiPO is fully charged, ~12.5v or similar
 *     - Push-out time while LiPO is depleted, storage voltage 11.5v or similar 
 *     - and retract time. 
 *     
 *    It is advisable to disconnect the ESC signal wire so that you don't have the motors draining power, and affecting your tune.
 *
 *    We need to change these values in the con fig screen 
 *      SP Hi, Solenoid PulseOnTimeHigh, default = 35;
 *      SP Low, Solenoid PulseOnTimeLow, default = 45;
 *      SP Ret, Solenoid PulseRetractTime = 45;
 * 
 *    Focus on the return rate first 
 *    Via config menu:-
 *      Change A ROF to Max, it is by default
 *      Change SP Hi, Solenoid Pulse On Time High = 150
 *      Change SP Low, Solenoid Pulse On Low = 150
 *      Change SP Ret, Solenoid Pulse Retract Time = 100
 *    Exit Config
 *    
 *    Set select fire to Full Auto 
 *    Start with PulseRetractTime, at 100ms,to see if the solenoid is fully retracting between shots. 
 *    You might be able to use your finger to feel it. Increase or reduce this number until you have a consistent and reliable retraction.
 *    n.b. solenoids will retract slower with a fully loaded magazine, so once you have found the number, add a bit extra to it (like 10ms) 
 *    To attchive the above may require multi edits
 *   
 *    Then focus on the fully charged pulse time. Fully charge your LiPO. Start reducing both PulseOnTimeHigh and PulseOnLow 
 *    (keep the numbers the same) until it's quick, but not so quick as it's short-stroking.
 *    Use your finger up the magwell to determine how hard the solenoid is hitting it. Keep reducing the time until it starts to feel weaker, 
 *    then back off. 
 *    Add a bit extra to it for safety margin. Write down this value.
 *
 *    For the low battery discharge time, bring the LiPO to storage charge. Then do the same thing as above.
 *
 *    Once you have both the PulseOnTimeHigh and PulseOnLow value, key them in and test the solenoid over a range of battery values. 
 *    Reducing the timing will make it faster, at the expense of reliability.
 *    
 *    Thous values: SP HI = 47, SP Low = 47, SP Ret = 27
 *    
 *
 * ----------------------------------------------------------------------------------------
 * Suggested ESC setting, the ones to change.
 * ----------------------------------------------------------------------------------------
 * Ramp Up Pwr: 150%
 * Low Voltage Protection: Off    //leave Off, motoro draw down adt startup can trigger this= unreliability
 * Motor Timing: 20 Deg
 * Min Trottel: 1040
 * Max Trottel: 1960              // motoro dependant knormaly go for 80% of max capability
 * Brake On Stop: Off             // This uses bat pwr and makes motors hot, can melt 3d prints if on
 * Led control Off x 3
 * Becon Delay: Off               // if not set to, off, ESC wil make a bleeping noise after no activity (downed dron). We might be hunting targets turn OFF
 * PWM FRQ: 48Khz
 * 
 * -----------------------------------------------------------------------------------------
 * Logo Splach Screen - How To
 * -----------------------------------------------------------------------------------------
 * Generate 128x64 Pixel BMP or PNG file (MS Paint works), edit to suit (White background, black details)
 * open browser on this https://javl.github.io/image2cpp/ (active link) use these settings:
 * Browse and select your file 
 * Background color: Black
 * Invert Image colors: off
 * Brightness 128
 * Out Put format: Arduino, Singal Bitmap
 * Draw mode: Vertical-1 bit per pixel
 * generate code & copy output
 * goto Logo.h in Arduino IDE
 * find bottom and past Ctrl-V
 * edit into the abvoe block , ie delete the hex and past in the new hex 
 * 
 *  
 * ------------------------------------------------------------------------ 
 * HISTORY
 * Originals files from:-
 * ------------------------------------------------------------------------ 
 * (c) 2019 Michael Ireland / Ireland Software
 * 
 * Creative Commons: Attribution, Non-Commercial
 * Free for Non-Commercial use
 * ------------------------------------------------------------------------
 * File: Nafduino FDL.ini and Caliburn 3.04.ini
 * v1.0 - Initial release
 * v1.1 - Adjusted the ADC for better throughput, and the screen refresh timing
 * 
 * Vx.x Richard Nicholson, from New Zealand. started to play with intention to stitch in Solenoid operation from Caliburn code
 * v2 Org code uses a HW shift reg console for encoder PBs. Encoder and switch working on nano HW, but primitive
 * v3 failed
 * v4 Encoder Working and Encoder Sw, Tidy up and document
 * v5 Another go at Encoder CLK/DT, working better with no INT, Process Econder in ProcessDebounce, yippy
 * v6 MAG Sensor IP working
 * v7 Trigger IP Working
 * v8 Start of Solenoid pusher changes (hardwork)
 * v9 Lot of stuff disableed, try anouther cut in
 * v10 cut in but not quite going
 * v11 life, its firing be it slow. probably due to debug code
 * v12 Working, tidy up and remove old code, add comments
 * V14 PulseOnTime fixed, Burst mode error count fixed
 * V15 Use Long press to get into Config, Short press to get out. Selectfire mode change via short click
 * V16 fix ROF issue, remove debug lines
 * v17 More Tidy Ups and documentation. Flip Screen, cos Thor got it up the wrong way :-), but easy fix and good for later reffferance
 * v18 Configuration menu tidy up and addition of solenoide tuning variables in config menu
 * v19 Tidy up code and comments
 * v20 Battery tip select and Battery Offset Calibration added to config menu
 * 
 *---------------------------------------------------------------------------------------------- 
 */

#include <EEPROM.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Wire.h>
#include "MenuHelpers.h"
#include "Logo.h"
#include "Constants.h"
// #include "Settings.h"
#include "Rotary.h"
#include "Trigger.h"
#include "Motor.h"
#include "Battery.h"
#include "Config.h"
#include "Oled.h"


//-------------------------------------------------------------------------------------------------------------------------

// Pin macros
/*
 * Read pin state
 * 
 * Valid for:
 * PB3/D11 - Mag Switch                        MAG_S

 * 
 */

#define GET_MAG_S ((PINB & 0b00001000) ? HIGH : LOW)  // PB3 this is what we want for mag Sensor

/*
 * Set Pin State
 * 
 * Valid for:
 * D5  PD5 P-RUN   // Pusher
 * D10 PB2/OC1B    // ESC Stage 1 
 * D9  PB1/OC1A    // ESC Stage 2 F-RUN
 */
#define SET_P_RUN_FULL_ON (PORTD |= 0b00100000)
#define SET_P_RUN_FULL_OFF (PORTD &= 0b11011111)



// Inputs
#define DebounceWindow 5          // Debounce Window = 5ms
#define RotaryDebounceWindow 100  // Report window for rotary encoder changes
#define RepollInterval 250        // Just check the buttons ever xxx just in case we missed an interrupt.

volatile bool MagOutChanged = false;  // ISR use


byte MagOutButtonState = BTN_HIGH;

byte ConsoleButtonLeftButtonState = BTN_LOW;    //Old consol buttons code left for referance
byte ConsoleButtonCenterButtonState = BTN_LOW;  //Old consol buttons code left for referance
byte ConsoleButtonRightButtonState = BTN_LOW;   //Old consol buttons code left for referance




volatile byte LastPIND = 0;  // Preload with PIND Prior to events happening

byte ConsoleAButtonAction = BTN_ACTION_NONE;  //Old consol buttons code left for referance
byte ConsoleBButtonAction = BTN_ACTION_NONE;  //Old consol buttons code left for referance
byte ConsoleCButtonAction = BTN_ACTION_NONE;  //Old consol buttons code left for referance

// Pusher Controls - solenoid
volatile bool RequestStop = false;  //
int PulseOnTime = 35;               // 35    50
int PulseOnTimeHigh = 35;           // 45    90
int PulseOnTimeLow = 45;            // 25    45
int PulseRetractTime = 45;          //
#define SOLENOID_CYCLE_IDLE 0
#define SOLENOID_CYCLE_PULSE 1
#define SOLENOID_CYCLE_RETRACT 2
#define SOLENOID_CYCLE_COOLDOWN 3
byte CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
unsigned long LastSolenoidCycleStarted = 0;
volatile bool PusherTickTock = false;  // Flag indicating a solenoid fire cyclce has completed
byte ROFAdjustment = 0;                // Used to calculate the next adjustment into
byte ROFAdjustmentA = 0;               // Used to store the Auto adjustment
byte ROFAdjustmentB = 0;               // Used to store the Burst adjustment
byte TargetDPS = 0;                    // Used to calculate the next adjustment into


// not uesed. referance
#define MAX_TRAVEL_TIME 1000                     // Old code left for referance
bool PusherRunning = false;                      // Old code left for referance
volatile unsigned long LastPusherResetTime = 0;  // Old code left for referance
bool JamDetected = false;                        // Old  code left for referance


// Firing Controls
int DartsToFire = 0;
#define FIRE_MODE_SINGLE 0
#define FIRE_MODE_BURST 1
#define FIRE_MODE_AUTO 2
#define FIRE_MODE_SAFE 3
#define FIRE_MODE_IDLE 4
byte CurrentFireMode = FIRE_MODE_IDLE;
byte TargetFireMode = FIRE_MODE_SINGLE;  //
bool RunFiringSequence = false;
bool StartNewFiringSequence = false;
bool ShutdownFiringSequence = false;
byte BurstSize = 3;
bool ExecuteFiring = false;  // Set to true when the Solenoid is supposed to move
int TimeBetweenShots = 0;    // Calculated to lower ROF


// Misc Controls
byte MagSize = 18;
int DartsInMag = 0;
bool EnteringConfig = false;
bool ExitingConfig = false;
bool UpdateAmmoCounter = false;  // Use this to signify when to update the ammo counter during a firing cycle.


// Button User interface. Note uesed left as example, cos its cool
#define BTN_SINGLE 0
#define BTN_BURST 1
#define BTN_AUTO 2
#define BTN_SAFE 3
#define BTN_PROFILE_A 4
#define BTN_PROFILE_B 5
#define BTN_NOTHING 6
#define BTN_IDX_LS 0
#define BTN_IDX_CS 1
#define BTN_IDX_RS 2
#define BTN_IDX_LL 3
#define BTN_IDX_CL 4
#define BTN_IDX_RL 5
#define BTN_IDX_ROT 6
byte ButtonActions[7] = { BTN_SINGLE, BTN_BURST, BTN_AUTO, BTN_PROFILE_A, BTN_NOTHING, BTN_PROFILE_B, BTN_SAFE };


// System
byte SystemMode = SYSTEM_MODE_NORMAL;


int SolenoidLowVoltage = (int)(BATTERY_3S_MIN * 10.0);
int SolenoidHighVoltage = (int)(BATTERY_3S_MAX * 10.0);



// -------------------------------------------------------------------------------------
ISR(PCINT0_vect)  // Rotary Encoder PD switch and MAG Sensor PORT B PCINT 3 & 4
{
  // Encoder Sw is PB4,
  if ((PINB & 0b00010000) != (LastPINB & 0b00010000)) {
    EncoderSwChanged = true;
  }

  // Mag Switch is PB3
  if ((PINB & 0b00001000) != (LastPINB & 0b000001000)) {
    MagOutChanged = true;
  }

  LastPINB = PINB;
}


// Profile Storage... I really should transact out of these.. But it allows for more flexability in a remote control scenario
byte CurrentProfile = 0;
struct ProfileDef {
  byte MotorSpeedFull = 50;               // Power: 50%
  byte ROFAdjustmentA = 0;                // A ROF Max    Full Auto rate
  byte ROFAdjustmentB = 0;                // B ROF Max    Burst rate
  byte BurstSize = 2;                     // Burst 2
  byte MagSize = 18;                      // Mag 18
  int AccelerateTime = 0;                 // Ramp Up 0 = fastest start up
  int DecelerateTime = 4000;              // Ramp Down
  int MotorStartDwellTime = 0;            // Dwell Up keeps it reving
  int MotorStopDwellTime = 200;           // Dwell Down
  int PulseOnTimeHigh = 35;               // SP High (Solenoid)
  int PulseOnTimeLow = 45;                // SP Low
  int PulseRetractTime = 45;              // SP Ret
                                          // ----------- NOT USED Referance code ---------------------------------------------
  byte BtnLS = BTN_NOTHING;               // Lft S: Single  (Console Button Left, Short press, single shot mode) BTN_SINGLE
  byte BtnLL = BTN_NOTHING;               // Lft L:Pro A    (Console Button Left, Long press, Select profile A) BTN_PROFILE_A
  byte BtnCS = BTN_NOTHING;               // Cen S: Burst   (Console Button Center, Short press, Burst Mode) BTN_BURST
  byte BtnCL = BTN_NOTHING;               // Cen L: Null    (Console Button Center, Long press, Null)
  byte BtnRS = BTN_NOTHING;               // Rgt S: Auto    (Console Button Right, Short press, Full Auto Mode ) BTN_AUTO
  byte BtnRL = BTN_NOTHING;               // Rgt L:Pro B    (Console Button Right, Long pree, Select profile B) BTN_PROFILE_B
  byte BtnRot = BTN_NOTHING;              // Rot L:Safe    (Rotary Encoder Button, Long press, Safe Mode) BTN_SAFE
                                          // ---------------------------------------------------------------------------------
  byte BatteyType = BATTERY_3S;           // Default is 3S battery. this Variable gets changed via the config menu
  int BatteryOffset = BATTERY_CALFACTOR;  // Battery Voltes offset/calibration, adjusted in config menu


  //int SolenoidHighVoltage = (int)(BATTERY_3S_MAX*10.0);     // these get changed when the bat type is changed, in config menu
  //int SolenoidLowVoltage = (int)(BATTERY_3S_MIN*10.0);      //
};
ProfileDef Profiles[2];



//============================================= SETUP =======================================
void setup() {
  Serial.begin(57600);  // Used for Debugging
  Serial.println();
  Serial.print(F("Booting | Ver: "));
  Serial.println(VERSION);  // Displat Version on Boot

  unsigned long BootStart = millis();

  InitDisplay();
  DisplayLogo();
  //--------------------- Config Interupt Vectors  ----------------------------------
  // PCINT Inputs
  // PB - PCINT Vector 0 ----PORT B --Encoder PB Switch & Mag Sensor
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);  // D12 PB4 PCINT4
  PCMSK0 |= _BV(PCINT4);
  pinMode(PIN_MAG_S, INPUT_PULLUP);  // D11 PB3 PCINT3
  PCMSK0 |= _BV(PCINT3);

  // PC - PCINT Vector 1 --- PORT C ---- Trigger
  pinMode(PIN_TRIGGER, INPUT_PULLUP);  // D16 PC2 PCINT10
  PCMSK1 |= _BV(PCINT10);


  LastPINB = PINB;                   // Ensure that we pre-load the LastPINB register to capture changes from NOW...
  LastPINC = PINC;                   // Ensure that we pre-load the LastPINC register to capture changes from NOW...
  PCICR |= _BV(PCIE0) | _BV(PCIE1);  // Activates control register for PCINT vector 0 & 1

  // External Interrupts for Rotary Encoder CLK-D2 & DT-D3 inputs
  pinMode(PIN_ENCODER_CLK, INPUT_PULLUP);
  EICRA |= _BV(ISC00);  // Triggers on edge
  EIMSK |= _BV(INT0);   // activates the interrupt

  pinMode(PIN_ENCODER_DT, INPUT_PULLUP);
  EICRA |= _BV(ISC10);  // Triggers on edge
  EIMSK |= _BV(INT1);   // activates the interrupt


  //-------------------------------- Setup ESC Stage 1 Output PIN 10 OC1B -------------------------------------------------
  // Setup Motor ESC Outputs
  Serial.println(F("Config PWM Ports"));
  pinMode(PIN_ESC, OUTPUT);
  digitalWrite(PIN_ESC, LOW);
  PORTD &= 0b11011111;  //PORTD?? its on PORT B2
  TCCR1A = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1B1);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 40000;
  UpdatePWM(1000);  //this value gets updated by ProcessMainMotors

  //-------------------------------------------------------------------------------------

  /*
  //--------------------------- Setup ESC Stage 2 Output PIN 9 OC1A --------------------------
  Serial.println( F("Configuring PWM Ports") );
  pinMode( PIN_ESC_2, OUTPUT );
  digitalWrite( PIN_ESC_2, LOW );
  TCCR1A = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1A1);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 40000;
  UpdatePWM( 1000 );            //this value gets updated by ProcessMainMotors
  */

  //----------------------------------------------------------------------------
  // Setup Pusher Outputs
  Serial.println(F("Configuring Pusher FET"));
  pinMode(PIN_PUSHER_RUN, OUTPUT);
  SET_P_RUN_FULL_OFF;  // binary load of the port to turn push OP off

  // Calculate the motor ramp rates
  CalculateRampRates();

  // Set the system mode to normal
  SystemMode = SYSTEM_MODE_NORMAL;

  // Setup the ADC
  ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX1) | (1 << MUX2);  // Use AVCC as the aref; Set MUX to ADC7
  ADCSRA = 0;
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // Turn on ADC, Interrupt enable, Prescalar to 128
  ADCSRB = 0;
  // SFIOR has ADTS0-2 already set to 0 for free-running mode.
  // Start ADC running
  ADCSRA |= (1 << ADSC);

  // Pre-charge the battery indicator
  for (byte c = 0; c < 8; c++) {
    ProcessBatteryMonitor();  // Check battery voltage occasionally
    delay(550);
  }

  // Wait for the sync - 2 seconds, but run the button debouncer to capture a trigger down state for EEPROM reset
  while (millis() - BootStart < 5000) {
    ProcessDebouncing();
    delay(10);
  }

  // --------------------- Load and apply profile configuration ------------------------------------------
  LoadEEPROM();
  UpdateProfile();

  // Now wait until the trigger is high, this forces a default of the eeprom
  Serial.println(F("Waiting for trigger safety"));
  Oled.clear();
  Oled.setFont(ZevvPeep8x16);
  Oled.setCursor(oled_adj + 40, 2);
  Oled.print(F("Finger"));
  Oled.setCursor(oled_adj + 35, 4);
  Oled.print(F("Off The"));
  Oled.setCursor(oled_adj + 35, 6);
  Oled.print(F("Trigger!"));
  while ((TriggerButtonState == BTN_LOW) || (TriggerButtonState == BTN_FELL)) {
    ProcessDebouncing();
    delay(10);
  }
  delay(10);

  // ---------------------------- Build tge menu system ----------------------------------
  BuildMenu();

  // We are done.
  Oled.clear();
  Oled.setFont(ZevvPeep8x16);
  Oled.setCursor(oled_adj + 40, 2);
  Oled.print(F("Booted"));

  Serial.println(F("Booted"));
}

//=================================================== LOAD EEPROM =========================================
void LoadEEPROM() {
  // Just dump the EEPROM into SRAM and worry about it later... Coz I'm a lazy fuck.
  byte* ProfileBaseAddress = &Profiles[0].MotorSpeedFull;
  for (byte c = 0; c < ADDR_SECTORL; c++) {
    *(ProfileBaseAddress + c) = EEPROM.read(c + ADDR_PROA_BASE);
  }
  ProfileBaseAddress = &Profiles[1].MotorSpeedFull;
  for (byte c = 0; c < ADDR_SECTORL; c++) {
    *(ProfileBaseAddress + c) = EEPROM.read(c + ADDR_PROB_BASE);
  }

  // Check for validity of data (range test)
  bool CorruptData = false;
  if ((Profiles[0].MotorSpeedFull < 30) || (Profiles[0].MotorSpeedFull > 100)) CorruptData = true;
  if ((Profiles[0].ROFAdjustmentA < 0) || (Profiles[0].ROFAdjustmentA > 150)) CorruptData = true;
  if ((Profiles[0].ROFAdjustmentB < 0) || (Profiles[0].ROFAdjustmentB > 150)) CorruptData = true;
  if ((Profiles[0].BurstSize < 2) || (Profiles[0].BurstSize > 99)) CorruptData = true;
  if ((Profiles[0].MagSize < 0) || (Profiles[0].MagSize > 99)) CorruptData = true;
  if ((Profiles[0].AccelerateTime < 0) || (Profiles[0].AccelerateTime > 5000)) CorruptData = true;
  if ((Profiles[0].DecelerateTime < 0) || (Profiles[0].DecelerateTime > 5000)) CorruptData = true;
  if ((Profiles[0].MotorStartDwellTime < 0) || (Profiles[0].MotorStartDwellTime > 5000)) CorruptData = true;
  if ((Profiles[0].MotorStopDwellTime < 0) || (Profiles[0].MotorStopDwellTime > 5000)) CorruptData = true;
  if ((Profiles[0].PulseOnTimeHigh < 0) || (Profiles[0].PulseOnTimeHigh > 1000)) CorruptData = true;
  if ((Profiles[0].PulseOnTimeLow < 0) || (Profiles[0].PulseOnTimeLow > 1000)) CorruptData = true;
  if ((Profiles[0].PulseRetractTime < 0) || (Profiles[0].PulseRetractTime > 1000)) CorruptData = true;
  if ((Profiles[0].BtnLS < 0) || (Profiles[0].BtnLS > 6)) CorruptData = true;
  if ((Profiles[0].BtnLL < 0) || (Profiles[0].BtnLL > 6)) CorruptData = true;
  if ((Profiles[0].BtnCS < 0) || (Profiles[0].BtnCS > 6)) CorruptData = true;
  if ((Profiles[0].BtnCL < 0) || (Profiles[0].BtnCL > 6)) CorruptData = true;
  if ((Profiles[0].BtnRS < 0) || (Profiles[0].BtnRS > 6)) CorruptData = true;
  if ((Profiles[0].BtnRL < 0) || (Profiles[0].BtnRL > 6)) CorruptData = true;
  if ((Profiles[0].BtnRot < 0) || (Profiles[0].BtnRot > 6)) CorruptData = true;
  if ((Profiles[0].BatteyType < BATTERY_3S) || (Profiles[0].BatteyType > BATTERY_4S)) CorruptData = true;
  if ((Profiles[0].BatteryOffset < -20) || (Profiles[0].BatteryOffset > 20)) CorruptData = true;

  if ((Profiles[1].MotorSpeedFull < 30) || (Profiles[1].MotorSpeedFull > 100)) CorruptData = true;
  if ((Profiles[1].ROFAdjustmentA < 0) || (Profiles[1].ROFAdjustmentA > 150)) CorruptData = true;
  if ((Profiles[1].ROFAdjustmentB < 0) || (Profiles[1].ROFAdjustmentB > 150)) CorruptData = true;
  if ((Profiles[1].BurstSize < 2) || (Profiles[1].BurstSize > 99)) CorruptData = true;
  if ((Profiles[1].MagSize < 0) || (Profiles[1].MagSize > 99)) CorruptData = true;
  if ((Profiles[1].AccelerateTime < 0) || (Profiles[1].AccelerateTime > 5000)) CorruptData = true;
  if ((Profiles[1].DecelerateTime < 0) || (Profiles[1].DecelerateTime > 5000)) CorruptData = true;
  if ((Profiles[1].MotorStartDwellTime < 0) || (Profiles[1].MotorStartDwellTime > 5000)) CorruptData = true;
  if ((Profiles[1].MotorStopDwellTime < 0) || (Profiles[1].MotorStopDwellTime > 5000)) CorruptData = true;
  if ((Profiles[1].PulseOnTimeHigh < 0) || (Profiles[1].PulseOnTimeHigh > 1000)) CorruptData = true;
  if ((Profiles[1].PulseOnTimeLow < 0) || (Profiles[1].PulseOnTimeLow > 1000)) CorruptData = true;
  if ((Profiles[1].PulseRetractTime < 0) || (Profiles[1].PulseRetractTime > 1000)) CorruptData = true;
  if ((Profiles[1].BtnLS < 0) || (Profiles[1].BtnLS > 6)) CorruptData = true;
  if ((Profiles[1].BtnLL < 0) || (Profiles[1].BtnLL > 6)) CorruptData = true;
  if ((Profiles[1].BtnCS < 0) || (Profiles[1].BtnCS > 6)) CorruptData = true;
  if ((Profiles[1].BtnCL < 0) || (Profiles[1].BtnCL > 6)) CorruptData = true;
  if ((Profiles[1].BtnRS < 0) || (Profiles[1].BtnRS > 6)) CorruptData = true;
  if ((Profiles[1].BtnRL < 0) || (Profiles[1].BtnRL > 6)) CorruptData = true;
  if ((Profiles[1].BtnRot < 0) || (Profiles[1].BtnRot > 6)) CorruptData = true;
  if ((Profiles[1].BatteyType < BATTERY_3S) || (Profiles[1].BatteyType > BATTERY_4S)) CorruptData = true;
  if ((Profiles[1].BatteryOffset < -20) || (Profiles[1].BatteryOffset > 20)) CorruptData = true;

  // Data is not valid, or the trigger was held on power-on for a reset (load default values)
  if ((TriggerButtonState == BTN_LOW) || CorruptData) {
    Serial.println(F("Something wrong with EEPROM or held trigger while booting"));

    // Create a fresh profile to use as a template n(get default values)
    ProfileDef TemplateProfile;

    // Just dump it over the old ones. If I was less lazy, i'd use memcpy :p
    byte* ProfileBaseAddress = &Profiles[0].MotorSpeedFull;
    byte* TemplateBaseAddress = &TemplateProfile.MotorSpeedFull;
    for (byte c = 0; c < ADDR_SECTORL; c++) {
      *(ProfileBaseAddress + c) = *(TemplateBaseAddress + c);
    }
    ProfileBaseAddress = &Profiles[1].MotorSpeedFull;
    for (byte c = 0; c < ADDR_SECTORL; c++) {
      *(ProfileBaseAddress + c) = *(TemplateBaseAddress + c);
    }

    // And flash it back to EEPROM
    ProfileBaseAddress = &Profiles[0].MotorSpeedFull;
    for (byte c = 0; c < ADDR_SECTORL; c++) {
      EEPROM.write(c + ADDR_PROA_BASE, *(ProfileBaseAddress + c));
    }
    ProfileBaseAddress = &Profiles[1].MotorSpeedFull;
    for (byte c = 0; c < ADDR_SECTORL; c++) {
      EEPROM.write(c + ADDR_PROB_BASE, *(ProfileBaseAddress + c));
    }
  }
}


//---------------------------------------------- Calculate Ramp Rates Subroutine -----------------------------------------------
// This is a boot time init sub to calcualte the Acceleration and deceleration ramp rates of the motors.
void CalculateRampRates() {
  long SpeedRange = (long)(MaxMotorSpeed - MinMotorSpeed) * 1000;  // Multiply by 1000 to give some resolution due to integer calculations
  if (AccelerateTime == 0) {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  } else {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if (DecelerateTime == 0) {
    MotorRampDownPerMS = SpeedRange;  // For instant acceleration
  } else {
    MotorRampDownPerMS = SpeedRange / DecelerateTime;  // Use when Decelerating
  }
}


// --------------------------------- Updates the PWM Timers -----------------------------------
void UpdatePWM(int NewSpeed) {
  NewSpeed = (NewSpeed * 2) + 2;  // Adjust for the prescalar
  OCR1B = NewSpeed;               // ESC 1 (Pin D10)
  //OCR1A = NewSpeed;                 // ESC 2 (Pin D9)
  //Serial.println ( "UpdatePWM");
}

//===================================== ProcessDebpounceing ==============================================
// Process the debouncing of the directly connected MCU inputs. Interupts set flags from ISR handlers to trigger bits in here
void ProcessDebouncing() {
  // Single call to millis() for better performance
  unsigned long CurrentMillis = millis();

  /*
   * ------- Trigger
   */
  static bool RunTriggerTimer = false;
  static unsigned long LastTriggerPress = 0;
  static byte TriggerDebounceState = 0;
  // Set up a repoll interval, just in case the interrupt is missed.
  if (CurrentMillis - LastTriggerPress > RepollInterval) TriggerChanged = true;
  // Move from edge to steady state
  if (TriggerButtonState == BTN_ROSE) TriggerButtonState = BTN_HIGH;
  if (TriggerButtonState == BTN_FELL) TriggerButtonState = BTN_LOW;
  if (TriggerChanged) {
    TriggerChanged = false;
    if (!RunTriggerTimer) {
      LastTriggerPress = CurrentMillis;

      TriggerDebounceState = 0b01010101;  // We need to give the button time to settle. This will track.
      RunTriggerTimer = true;
    }
  }
  if (RunTriggerTimer && (CurrentMillis - LastTriggerPress > DebounceWindow)) {
    TriggerDebounceState = (TriggerDebounceState << 1) | ((PIND >> 6) & 0b00000001);  // Shift the register pin to the left, Shift the pin result to the right most position, and tack it onto the debounce state. Ensure that only the last position can be a 1.

    if ((TriggerDebounceState == 0) || (TriggerDebounceState == 255))  // All 0's or all 1's. This means we have settled.
    {
      RunTriggerTimer = false;

      if (GET_FIRE_T) {
        if (TriggerButtonState != BTN_HIGH) TriggerButtonState = BTN_ROSE;
      } else {
        if (TriggerButtonState != BTN_LOW) TriggerButtonState = BTN_FELL;
      }
    }
  }

  /*
   * --------- Rotary Encoder
   */
  // Encoder DT
  if (ConsoleRotaryAButtonState == BTN_ROSE) ConsoleRotaryAButtonState = BTN_HIGH;
  if (ConsoleRotaryAButtonState == BTN_FELL) ConsoleRotaryAButtonState = BTN_LOW;
  static bool LastConsoleRotaryAState = true;
  bool ConsoleRotaryAState = (GET_ENCODER_DT);
  if (ConsoleRotaryAState != LastConsoleRotaryAState) {
    LastConsoleRotaryAState = ConsoleRotaryAState;
    if (ConsoleRotaryAState) {
      ConsoleRotaryAButtonState = BTN_FELL;
    } else {
      ConsoleRotaryAButtonState = BTN_ROSE;
    }
  }

  // Encoder CLK
  if (ConsoleRotaryBButtonState == BTN_ROSE) ConsoleRotaryBButtonState = BTN_HIGH;
  if (ConsoleRotaryBButtonState == BTN_FELL) ConsoleRotaryBButtonState = BTN_LOW;
  static bool LastConsoleRotaryBState = true;
  bool ConsoleRotaryBState = (GET_ENCODER_CLK);

  if (ConsoleRotaryBState != LastConsoleRotaryBState) {
    LastConsoleRotaryBState = ConsoleRotaryBState;
    if (ConsoleRotaryBState) {
      ConsoleRotaryBButtonState = BTN_FELL;
    } else {
      ConsoleRotaryBButtonState = BTN_ROSE;
    }
  }

  // -------------------------- Rotary Encoder Switch handling ------------
  /*
   * Rotary Encoder PB
   */
  if (ConsoleRotaryCButtonState == BTN_ROSE) ConsoleRotaryCButtonState = BTN_HIGH;
  if (ConsoleRotaryCButtonState == BTN_FELL) ConsoleRotaryCButtonState = BTN_LOW;
  static bool LastConsoleRotaryCState = true;
  bool ConsoleRotaryCState = (GET_ENCODER_SW);

  if (ConsoleRotaryCState != LastConsoleRotaryCState) {
    EncoderSwChanged = false;
    LastConsoleRotaryCState = ConsoleRotaryCState;
    if (ConsoleRotaryCState)
      ConsoleRotaryCButtonState = BTN_FELL;
    else
      ConsoleRotaryCButtonState = BTN_ROSE;
  }
  // -----------------------------------------------------------------
  /*
   * MagSwitch
   */
  static bool RunMagSwitchTimer = false;
  static unsigned long LastMagSwitchPress = 0;
  static byte MagSwitchDebounceState = 0;
  // Set up a repoll interval, just in case the interrupt is missed.
  if (CurrentMillis - LastMagSwitchPress > RepollInterval) MagOutChanged = true;
  // Move from edge to steady state
  if (MagOutButtonState == BTN_ROSE) MagOutButtonState = BTN_HIGH;
  if (MagOutButtonState == BTN_FELL) MagOutButtonState = BTN_LOW;
  if (MagOutChanged) {
    MagOutChanged = false;
    if (!RunMagSwitchTimer) {
      LastMagSwitchPress = CurrentMillis;

      MagSwitchDebounceState = 0b01010101;  // We need to give the button time to settle. This will track.
      RunMagSwitchTimer = true;
    }
  }
  if (RunMagSwitchTimer && (CurrentMillis - LastMagSwitchPress > DebounceWindow)) {
    MagSwitchDebounceState = (MagSwitchDebounceState << 1) | ((PIND >> 7) & 0b00000001);  // Shift the register pin to the left, Shift the pin result to the right most position, and tack it onto the debounce state. Ensure that only the last position can be a 1.

    if ((MagSwitchDebounceState == 0) || (MagSwitchDebounceState == 255))  // All 0's or all 1's. This means we have settled.
    {
      RunMagSwitchTimer = false;

      if (GET_MAG_S) {
        if (MagOutButtonState != BTN_HIGH) MagOutButtonState = BTN_ROSE;

      } else {
        if (MagOutButtonState != BTN_LOW) MagOutButtonState = BTN_FELL;
      }
    }
  }
}


//=============================================================================================================================
//                      Main loop. Run the blaster here.
//=============================================================================================================================
void loop() {
  ProcessDebouncing();            // Process the pin input, and handle any debouncing
  ProcessButtons();               // Where interperation is necessary, handle it here Trigger is being done here
  ProcessBatteryMonitor();        // Check battery voltage occasionally
  ProcessMagRelease();            // Handle when a magazing is dropped out
  ProcessSystemMode();            // Find out what the system should be doing
  ProcessConsoleButtonManager();  // Work out what to do based on the console button input
  ProcessDisplay();               // Update the Oled display

  // Detected a change to the command. Reset the last speed change timer.
  if (PrevCommandRev != CommandRev) {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }
  ProcessSpeedControl();  // Process speed control to resolve a % target
  ProcessMotorSpeed();    // Calculate the new motor speed in terms of a PWM signal
  ProcessMainMotors();    // Send the speed to the ESC

  ProcessFiringControl();  // Handle the cycle control here

  ProcessPusherControl();  // Handle the pusher management here

}  // ======================= End Main Loop =============================================


//---------------------------------------------------------------------------------------------------
// this process butons after debounce, ie long and short clicks, and encoder value
void ProcessConsoleButtonManager() {

  // Decode the rotary
  static unsigned long LastRotaryChange = 0;
  RotaryTurned = ROTARY_NONE;

  bool ConsoleRotaryAValue = (ConsoleRotaryAButtonState == BTN_ROSE) || (ConsoleRotaryAButtonState == BTN_HIGH);
  bool ConsoleRotaryBValue = (ConsoleRotaryBButtonState == BTN_ROSE) || (ConsoleRotaryBButtonState == BTN_HIGH);
  static bool LCAV = false;  //Last Console A Value
  static bool LCBV = false;  //Last Console B Value
  bool Change = false;

  if (ConsoleRotaryAValue != LCAV) {
    Change = true;
  }

  if (ConsoleRotaryBValue != LCBV) {
    Change = true;
  }

  if (Change)

  // RotaryTurned =, is the output of this bit, hock in for encoder signal - RICH
  {
    if (!ConsoleRotaryAValue && !ConsoleRotaryBValue) {
      if (!LCAV && LCBV) {
        RotaryTurned = ROTARY_CW;
      }
      if (LCAV && !LCBV) {
        RotaryTurned = ROTARY_CCW;
      }
    }

    LCAV = ConsoleRotaryAValue;
    LCBV = ConsoleRotaryBValue;
  }

  // Handle the buttons --------------------------------------
  static unsigned long LastConsoleRotaryDownTime = 0;
  static bool LastConsoleRotaryValue = false;
  Change = false;
  ConsoleRotaryButtonAction = BTN_ACTION_NONE;
  bool ConsoleRotaryValue = (ConsoleRotaryCButtonState == BTN_ROSE) || (ConsoleRotaryCButtonState == BTN_HIGH);
  if (LastConsoleRotaryValue != ConsoleRotaryValue) {
    Change = true;
    LastConsoleRotaryValue = ConsoleRotaryValue;
  }
  if (Change) {
    if (!ConsoleRotaryValue) {
      if (millis() - LastConsoleRotaryDownTime > BUTTON_PRESS_JUNK) {
        // Junk press.. Ignore
      } else if (millis() - LastConsoleRotaryDownTime > BUTTON_PRESS_LONG) {
        ConsoleRotaryButtonAction = BTN_ACTION_LONG;
      } else if (millis() - LastConsoleRotaryDownTime > BUTTON_PRESS_SHORT) {
        ConsoleRotaryButtonAction = BTN_ACTION_SHORT;
      }
    }
    if (ConsoleRotaryValue) {
      LastConsoleRotaryDownTime = millis();
    }
  }

  // ------------------------------ Not used ----------------
  static unsigned long LastConsoleADownTime = 0;
  static bool LastConsoleAValue = false;
  Change = false;
  ConsoleAButtonAction = BTN_ACTION_NONE;
  bool ConsoleAValue = (ConsoleButtonLeftButtonState == BTN_ROSE) || (ConsoleButtonLeftButtonState == BTN_HIGH);
  if (LastConsoleAValue != ConsoleAValue) {
    Change = true;
    LastConsoleAValue = ConsoleAValue;
  }
  if (Change) {
    if (!ConsoleAValue) {
      if (millis() - LastConsoleADownTime > BUTTON_PRESS_JUNK) {
        // Junk press.. Ignore
      } else if (millis() - LastConsoleADownTime > BUTTON_PRESS_LONG) {
        ConsoleAButtonAction = BTN_ACTION_LONG;
      } else if (millis() - LastConsoleADownTime > BUTTON_PRESS_SHORT) {
        ConsoleAButtonAction = BTN_ACTION_SHORT;
      }
    }
    if (ConsoleAValue) {
      LastConsoleADownTime = millis();
    }
  }

  // ----------------- Not used ------------------------
  static unsigned long LastConsoleBDownTime = 0;
  static bool LastConsoleBValue = false;
  Change = false;
  ConsoleBButtonAction = BTN_ACTION_NONE;
  bool ConsoleBValue = (ConsoleButtonCenterButtonState == BTN_ROSE) || (ConsoleButtonCenterButtonState == BTN_HIGH);
  if (LastConsoleBValue != ConsoleBValue) {
    Change = true;
    LastConsoleBValue = ConsoleBValue;
  }
  if (Change) {
    if (!ConsoleBValue) {
      if (millis() - LastConsoleBDownTime > BUTTON_PRESS_JUNK) {
        // Junk press.. Ignore
      } else if (millis() - LastConsoleBDownTime > BUTTON_PRESS_LONG) {
        ConsoleBButtonAction = BTN_ACTION_LONG;
      } else if (millis() - LastConsoleBDownTime > BUTTON_PRESS_SHORT) {
        ConsoleBButtonAction = BTN_ACTION_SHORT;
      }
    }
    if (ConsoleBValue) {
      LastConsoleBDownTime = millis();
    }
  }

  // ------------------------- Not used ----------------------------
  static unsigned long LastConsoleCDownTime = 0;
  static bool LastConsoleCValue = false;
  Change = false;
  ConsoleCButtonAction = BTN_ACTION_NONE;
  bool ConsoleCValue = (ConsoleButtonRightButtonState == BTN_ROSE) || (ConsoleButtonRightButtonState == BTN_HIGH);
  if (LastConsoleCValue != ConsoleCValue) {
    Change = true;
    LastConsoleCValue = ConsoleCValue;
  }
  if (Change) {
    if (!ConsoleCValue) {
      if (millis() - LastConsoleCDownTime > BUTTON_PRESS_JUNK) {
        // Junk press.. Ignore
      } else if (millis() - LastConsoleCDownTime > BUTTON_PRESS_LONG) {
        ConsoleCButtonAction = BTN_ACTION_LONG;
      } else if (millis() - LastConsoleCDownTime > BUTTON_PRESS_SHORT) {
        ConsoleCButtonAction = BTN_ACTION_SHORT;
      }
    }
    if (ConsoleCValue) {
      LastConsoleCDownTime = millis();
    }
  }

  // Execute controls per the button map,
  if (SystemMode == SYSTEM_MODE_NORMAL) {
    if (ConsoleCButtonAction == BTN_ACTION_SHORT)
      PerformButtonAction(ButtonActions[BTN_IDX_RS]);
    else if (ConsoleBButtonAction == BTN_ACTION_SHORT)
      PerformButtonAction(ButtonActions[BTN_IDX_CS]);
    else if (ConsoleAButtonAction == BTN_ACTION_SHORT)
      PerformButtonAction(ButtonActions[BTN_IDX_LS]);
    else if (ConsoleCButtonAction == BTN_ACTION_LONG)
      PerformButtonAction(ButtonActions[BTN_IDX_RL]);
    else if (ConsoleBButtonAction == BTN_ACTION_LONG)
      PerformButtonAction(ButtonActions[BTN_IDX_CL]);
    else if (ConsoleAButtonAction == BTN_ACTION_LONG)
      PerformButtonAction(ButtonActions[BTN_IDX_LL]);
    else if (ConsoleRotaryButtonAction == BTN_ACTION_SHORT)
      ProcessSelectFire();  //Changinging TargetFireMode here sets the fire mode
                            //else if( ConsoleRotaryButtonAction == BTN_ACTION_LONG )
                            //  PerformButtonAction( ButtonActions[ BTN_IDX_ROT ] );
  }
}


//------------------------------------------------------------------------------------------
// Perform the requested mapped function
void PerformButtonAction(byte ActionIDX) {
  switch (ActionIDX) {
    case BTN_SINGLE:
      //TargetFireMode = FIRE_MODE_SINGLE;
      break;
    case BTN_BURST:
      //TargetFireMode = FIRE_MODE_BURST;
      break;
    case BTN_AUTO:
      //TargetFireMode = FIRE_MODE_AUTO;
      break;
    case BTN_SAFE:
      //TargetFireMode = FIRE_MODE_SAFE;
      break;
    case BTN_PROFILE_A:
      CurrentProfile = 0;
      UpdateProfile();
      break;
    case BTN_PROFILE_B:
      CurrentProfile = 1;
      UpdateProfile();
      break;
    case BTN_NOTHING:
    default:
      // Do nothing
      break;
  }
}


// -------------------------------- Process Select Fire -------------------------------------------
// Set the Select Fire Mode. From normal run mode screen. Short Encoder click
void ProcessSelectFire() {
  static byte SelectFireClk = 0;
  SelectFireClk += 1;
  if (SelectFireClk == 4)  //trap clicks 1-3 range
  {
    SelectFireClk = 1;
  }
  if (SelectFireClk == 1) {
    TargetFireMode = FIRE_MODE_SINGLE;
  }
  if (SelectFireClk == 2) {
    TargetFireMode = FIRE_MODE_BURST;
  }
  if (SelectFireClk == 3) {
    TargetFireMode = FIRE_MODE_AUTO;
  }
}


//---------------------------------------- PROCESS SYTEM MODE ---------------------------------------------
// Process the main system state.
void ProcessSystemMode() {
  bool MagReleasePressed = ((MagOutButtonState == BTN_LOW) || (MagOutButtonState == BTN_FELL));

  //if( ((SystemMode == SYSTEM_MODE_NORMAL) || (SystemMode == SYSTEM_MODE_MAGOUT)) && ( ConsoleRotaryButtonAction == BTN_ACTION_SHORT ) && !ExitingConfig )
  if (((SystemMode == SYSTEM_MODE_NORMAL) || (SystemMode == SYSTEM_MODE_MAGOUT)) && (ConsoleRotaryButtonAction == BTN_ACTION_LONG) && !ExitingConfig) {
    EnteringConfig = true;
    SystemMode = SYSTEM_MODE_CONFIG;
  } else if (JamDetected) {
    SystemMode = SYSTEM_MODE_JAM;
    ExitingConfig = false;
  } else if (BatteryFlat) {
    SystemMode = SYSTEM_MODE_LOWBATT;
    ExitingConfig = false;
  } else if (!MagReleasePressed) {
    if ((SystemMode != SYSTEM_MODE_CONFIG) || ExitingConfig) {
      SystemMode = SYSTEM_MODE_MAGOUT;
      ExitingConfig = false;
    }
  } else if ((SystemMode != SYSTEM_MODE_CONFIG) || ExitingConfig) {
    SystemMode = SYSTEM_MODE_NORMAL;
    ExitingConfig = false;
  }
}


//------------------------------------- PROCESS FIRING CONTROL --------------------------------------------
void ProcessFiringControl()  // Handle the high-level firing logic
{
#define FS_STAGE_IDLE 0
#define FS_STAGE_REV 1
#define FS_STAGE_FIRING 2
#define FS_STAGE_DECEL 3
  static byte FiringSequenceStage = FS_STAGE_IDLE;
  static byte LastSystemMode = 99;
  static bool PreviousRunFiringSequence = true;
  static unsigned long RevStart = 0;
  static unsigned long DecelStart = 0;

  int ROFToLoad = 00;  // Calculate the ROF to load into the system, this is TargetDPS in solenoide code

  int StartLag = MOTOR_SPINUP_LAG + AccelerateTime;  // Calculate the lag for the motor start

  if (SystemMode != SYSTEM_MODE_NORMAL) {
    if (StartNewFiringSequence)
      StartNewFiringSequence = false;  // Abort a new cycle
    CommandRev = COMMAND_REV_NONE;
    if (RunFiringSequence) {
      switch (FiringSequenceStage) {
        case FS_STAGE_REV:
          // Bypass the firing sequence and start decelerating
          FiringSequenceStage = FS_STAGE_DECEL;
          DecelStart = 0;
          break;
        case FS_STAGE_FIRING:
          // Set one dart into the queue if there is more than one
          if (DartsToFire > 1) DartsToFire = 1;
          break;
        case FS_STAGE_DECEL:
          // Already decelerating
          break;
        default:
          break;  // Idle
      }
    }
  }

  if (StartNewFiringSequence) {
    StartNewFiringSequence = false;
    if ((SystemMode != SYSTEM_MODE_NORMAL)) {
      return;  // Don't start a new firing sequence if out of the right mode.
    }
    if (RunFiringSequence) {
      switch (FiringSequenceStage) {
        case FS_STAGE_REV:
          // Carry on
          break;
        case FS_STAGE_FIRING:
          // Add more darts to the queue
          switch (CurrentFireMode) {
            case FIRE_MODE_SINGLE:  // Add one to the queue
              DartsToFire++;
              break;
            case FIRE_MODE_BURST:
              DartsToFire = BurstSize;
              break;
            case FIRE_MODE_AUTO:
              DartsToFire = 99;
              break;
            default:  // Idle and Safe Mode
              break;
          }
          break;
        case FS_STAGE_DECEL:
          // Start again from scratch, but factor in the current motor speed
          if (TargetFireMode != FIRE_MODE_SAFE) {
            switch (CurrentFireMode) {
              case FIRE_MODE_SINGLE:  // Add one to the queue
                ROFToLoad = 0;
                break;
              case FIRE_MODE_BURST:
                ROFToLoad = ROFAdjustmentB;
                break;
              case FIRE_MODE_AUTO:
                ROFToLoad = ROFAdjustmentA;
                break;
              default:  // Idle and Safe Mode
                break;
            }
            FiringSequenceStage = FS_STAGE_REV;
            RequestStop = false;
            ROFAdjustment = ROFToLoad;
            CommandRev = COMMAND_REV_FULL;                               // this is ESC kick off
            RevStart = millis() + MotorStartDwellTime + (StartLag / 2);  // Basic ignore the user dwell time, and have half of the startup latency
            CurrentFireMode = TargetFireMode;
            //Serial.println ("REV_FULL-1");
          }
          // Add one in single shot mode
          // Reset to Burst in burst mode
          // Set to 99 in Auto mode
          switch (CurrentFireMode) {
            case FIRE_MODE_SINGLE:  // Add one to the queue
              DartsToFire = 1;
              break;
            case FIRE_MODE_BURST:
              DartsToFire = BurstSize;
              break;
            case FIRE_MODE_AUTO:
              DartsToFire = 99;
              break;
            default:  // Idle and Safe Mode
              break;
          }
          break;
        default:
          // We shouldn't be here, but we were probably idle.
          break;
      }
    } else {
      if (TargetFireMode != FIRE_MODE_SAFE) {
        RunFiringSequence = true;
        FiringSequenceStage = FS_STAGE_REV;
        RevStart = 0;
        CurrentFireMode = TargetFireMode;
      }
    }
  }

  if (ShutdownFiringSequence) {
    // Only need for auto, reset dart counter to 1 and let it fly out naturally
    ShutdownFiringSequence = false;
    if (CurrentFireMode == FIRE_MODE_AUTO) {
      DartsToFire = 1;
    }
  }

  if (RunFiringSequence) {
    if (FiringSequenceStage == FS_STAGE_REV) {
      if (RevStart == 0) {
        // Init
        switch (CurrentFireMode) {
          case FIRE_MODE_SINGLE:  // Add one to the queue
            ROFToLoad = 0;
            break;
          case FIRE_MODE_BURST:
            ROFToLoad = ROFAdjustmentB;
            break;
          case FIRE_MODE_AUTO:
            ROFToLoad = ROFAdjustmentA;
            break;
          default:  // Idle and Safe Mode
            break;
        }
        RequestStop = false;
        ROFAdjustment = ROFToLoad;
        CommandRev = COMMAND_REV_FULL;  //ESC trigger, kicks off void ProcessSpeedControl
        //Serial.println ("REV_FULL-2");
        RevStart = millis();
        switch (CurrentFireMode) {
          case FIRE_MODE_SINGLE:  // Add one to the queue
            DartsToFire = 1;
            break;
          case FIRE_MODE_BURST:
            DartsToFire = BurstSize;
            break;
          case FIRE_MODE_AUTO:
            DartsToFire = 99;
            break;
          default:  // Idle and Safe Mode
            break;
        }
      } else if (millis() - RevStart > (StartLag + MotorStartDwellTime))  // We have waited long enough for the motor to ramp
      {
        //Serial.println ("waited2longGo");                               // start pusher firing sequence
        RevStart = 0;
        FiringSequenceStage = FS_STAGE_FIRING;
        LastSolenoidCycleStarted = millis();
        //Serial.println ("SOLENOID STARTED");
        PulseOnTime = map((int)(BatteryCurrentVoltage * 10), SolenoidHighVoltage, SolenoidLowVoltage, PulseOnTimeLow, PulseOnTimeHigh);
        if (PulseOnTime < 0)
          PulseOnTime = 0;
        ExecuteFiring = true;
      }
    } else if (FiringSequenceStage == FS_STAGE_FIRING) {
      if (RequestStop)  // Want to stop - move from firing state to stop state
      {
        FiringSequenceStage = FS_STAGE_DECEL;
        DecelStart = 0;
      }
      if (CommandRev == COMMAND_REV_NONE)  // Somehow the motors aren't spinning
      {
        RequestStop = true;
        FiringSequenceStage = FS_STAGE_DECEL;
        DecelStart = 0;
      }
    } else if (FiringSequenceStage == FS_STAGE_DECEL) {
      if (DecelStart == 0)  // Initiate shutdown
      {
        DecelStart = millis();
      } else if (((millis() - DecelStart > MotorStopDwellTime) || (millis() - DecelStart > 5000)) && !(CommandRev == COMMAND_REV_NONE)) {
        CommandRev = COMMAND_REV_NONE;
      } else if (CurrentMotorSpeed == MinMotorSpeed)  // Shutdown complete
      {
        FiringSequenceStage = FS_STAGE_IDLE;
        RunFiringSequence = false;
        DecelStart = 0;
        CurrentFireMode = FIRE_MODE_IDLE;
      }
    }
  } else {
    CommandRev = COMMAND_REV_NONE;
  }
}
//------------------------------------ END ProcessFiringControl  --------------------


//===================================== Process Pusher Control ----------------------
void ProcessPusherControl() {
  static bool PreviousPusherTickTock = true;  // Flag indicating a solenoid firecycle has been completed

  if (!ExecuteFiring)  // Just skip if there is no firing to execute
  {
    return;
  }

  // this does Darts and Mag Counter, its trigger via the flag which is set after a solenoid fire cycle, has been completed
  if (PusherTickTock != PreviousPusherTickTock)  // Pusher has changed positions, a shot has been fired
  {
    PreviousPusherTickTock = PusherTickTock;  //  Recoder flag status
    PusherTickTock = false;                   //  Reset fired flag
    if (PreviousPusherTickTock)               //  Pusher is out, we have fired
    {
      DartsToFire--;
      if (MagSize == 0) {
        if (DartsInMag < 255) DartsInMag++;  // Handle the magazine status
      } else {
        if (DartsInMag >= 1) DartsInMag--;  // Handle the magazine status
      }
    } else {
      UpdateAmmoCounter = true;  // We're in the home position, so spend the time to update the OLED.
    }
  }  //------- End if PusherTickTock

  /*
  //---------------------------------- NOT NEEDED, Old FDL code left as example trap, links to display 'Jam"
  if( (millis() - LastPusherResetTime > MAX_TRAVEL_TIME) && (PusherRunning) ) // Jam detected. not triggering this timer any more
  {
    Serial.print( "Jam" );
    DartsToFire = 0;
    RequestStop = true;
    JamDetected = true;
    //PusherStatus = PUSHER_STATUS_STOPPED;
  }
  */

  // cheek some stuff
  //-------------------------------------------------------
  // Initiate Pusher Shutdown, if run out of darts
  if (DartsToFire < 1) {
    RequestStop = true;  //FIX later, triger shut down. not effecting anything
    Serial.println("PC_RequestStop");
  }

  // --------------------------------------------------
  // This is an error condition. Just kind pretend it never happens, but allow for debugging code to catch it.
  if (DartsToFire < 0) {
    //Serial.println( "%%%% WTF %%%%" );
    DartsToFire = 0;
  }

  // -------------------------------------------------------
  // Calculate duty cycle whenever the target changes (this is ROF)
  static byte PrevTargetDPS = 0;
  static bool ResetPulseTimer = true;

  TargetDPS = ROFAdjustment;  //link old and new code TargetDPS can be replaced with ROFAdjustment
  if (PrevTargetDPS != TargetDPS) {
    PrevTargetDPS = TargetDPS;
    if (TargetDPS == 99)  // Full rate
    {
      TimeBetweenShots = 0;
    } else {
      int PulseOverhead = PulseOnTime + PulseRetractTime;
      int TotalPulseOverhead = PulseOverhead * TargetDPS;
      int FreeMS = TotalPulseOverhead - 1000;
      TimeBetweenShots = FreeMS / TargetDPS;
      if (FreeMS <= 0) {
        TimeBetweenShots = 0;  // Pusher won't achieve this rate
      } else {
        TimeBetweenShots = FreeMS / TargetDPS;
      }
    }
  }

  // We actually have nothing to do.
  if (CurrentFireMode == FIRE_MODE_IDLE) {
    //Serial.println( "PC_FireMode IDEL" );
    return;  // Solenoid is idling.
  }

  // We are apparently supposed to fire 0 darts... Typically for end-of-firing scenarios. This is the exit point
  if ((DartsToFire == 0) && (CurrentFireMode != FIRE_MODE_IDLE)) {
    CurrentFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    SET_P_RUN_FULL_OFF;
    ExecuteFiring = false;
    RequestStop = true;
    //Serial.println( "PC_Finished" );
    return;
  }

  // Last check to ensure the motors are running before we send a dart into them
  if ((CommandRev == COMMAND_REV_NONE) && (SystemMode == SYSTEM_MODE_NORMAL)) {
    CurrentFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    SET_P_RUN_FULL_OFF;
    ExecuteFiring = false;
    RequestStop = true;
    //Serial.println( "PC_Rev None" );
    return;
  }

  // Pulse solenoid on high
  if ((millis() - LastSolenoidCycleStarted) < PulseOnTime) {
    //Serial.print ( "PulseOnTime ");
    //Serial.println (PulseOnTime);       //Calculated number handy to see
    //Serial.println( "PULSE" );
    if (CurrentSolenoidCyclePosition != SOLENOID_CYCLE_PULSE) {
      if ((SystemMode != SYSTEM_MODE_NORMAL))  // Don't fire unless the system mode is normal
      {
        DartsToFire = 0;
        return;
      }
    }
    if (ResetPulseTimer) {
      ResetPulseTimer = false;
      LastSolenoidCycleStarted = millis();
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_PULSE;
    SET_P_RUN_FULL_ON;
    return;
  }

  // Release solenoid for retraction
  if ((millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime)) {
    //Serial.println( "RETRACT" );
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_RETRACT;
    SET_P_RUN_FULL_OFF;
    return;
  }

  // Wait for the Global Cool Down... i.e. ROF adjustment
  if ((millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime + TimeBetweenShots)) {
    //Serial.println( "COOL" );
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_COOLDOWN;
    SET_P_RUN_FULL_OFF;
    return;
  }

  // We have completed a single solenoid cycle
  CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;  // Return to idle, ready for the next shot
  LastSolenoidCycleStarted = millis();
  ResetPulseTimer = true;
  PusherTickTock = true;  // Flag Shot fired, for DartsToFire/Amo Counter update, //DartsToFire -= 1;
  Serial.println("Bang!");
}
// ------------------------------------ End ProcessPusherControl  ------------------------------

//-------------------------------- PROCESS MAIN MOTORS ------------------
// Update the motors with the new speed
void ProcessMainMotors() {
  static int PreviousMotorSpeed = MinMotorSpeed;

  if (PreviousMotorSpeed != CurrentMotorSpeed) {
    if (CurrentMotorSpeed > MaxMotorSpeed) {
      UpdatePWM(MaxMotorSpeed);
      //Serial.println(MaxMotorSpeed);
    } else {
      UpdatePWM(CurrentMotorSpeed);
      //Serial.println (CurrentMotorSpeed);
    }
    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}


//----------------------------------PROCESS BUTTONS ------------------------
void ProcessButtons() {
  // Button Controls
  //Trigger
  if (TriggerButtonState == BTN_FELL) {
    StartNewFiringSequence = true;
  } else if (TriggerButtonState == BTN_ROSE) {
    if (RunFiringSequence) {
      ShutdownFiringSequence = true;
    }
  }
}


//------------------------------------------------------------------------
void ProcessMagRelease() {
  static bool LastMagReleasePressed = false;
  bool MagReleasePressed = ((MagOutButtonState == BTN_LOW) || (MagOutButtonState == BTN_FELL));
  if (LastMagReleasePressed != MagReleasePressed)  // Detect a change in the status quo
  {
    if (MagReleasePressed)  // A mag has been inserted
    {
      DartsInMag = MagSize;
    } else  // Mag has been dropped
    {
      DartsInMag = 0;
      JamDetected = false;
    }
  }
  LastMagReleasePressed = MagReleasePressed;
}


//==========================================================================================
void ProcessDisplay() {
  static int LastSystemMode = 99;
  bool ClearScreen = false;

  if (LastSystemMode != SystemMode) {
    ClearScreen = true;
    LastSystemMode = SystemMode;
  }

  Display_ScreenHeader(ClearScreen);
  switch (SystemMode) {
    case SYSTEM_MODE_MAGOUT:
      Display_MagOut(ClearScreen);
      break;
    case SYSTEM_MODE_CONFIG:
      Display_Config(ClearScreen);
      break;
    case SYSTEM_MODE_LOWBATT:
      Display_LowBatt(ClearScreen);
      break;
    case SYSTEM_MODE_JAM:
      Display_Jam(ClearScreen);
      break;
    default:
      Display_Normal(ClearScreen);
      break;
  }
}


//-------------------------------------------- DISPLAY MAG OUT SCREEN ------------------------
void Display_MagOut(bool ClearScreen) {
  if (ClearScreen) {
    Oled.setFont(ZevvPeep8x16);
    Oled.setCursor(oled_adj + 0, 2);
    Oled.print(F("################\n"));
    Oled.print(F("# MAG DROPPED! #\n"));
    Oled.print(F("################"));
  }
}

//---------------------------------------------- DISPALY DEFUALT SCREEN ---------------------
void Display_Normal(bool ClearScreen) {
  char Buffer[4];
  static int LastDartsInMag = 99;
  static byte LastTargetFireMode = 99;
  static byte LastDisplayDPS = 199;
  static byte LastMotorSpeedFull = 199;
  static byte LastBurstSize = 99;
  static unsigned long LastRefresh = 0;
  if (ClearScreen || (TargetFireMode != LastTargetFireMode) || (BurstSize != LastBurstSize)) {
    Oled.setFont(ZevvPeep8x16);
    Oled.setCursor(oled_adj + 0, 2);
    if (TargetFireMode == FIRE_MODE_SINGLE) {
      Oled.print(F("Single    "));
    } else if (TargetFireMode == FIRE_MODE_AUTO) {
      Oled.print(F("Full Auto "));
    } else if (TargetFireMode == FIRE_MODE_SAFE) {
      Oled.print(F("** SAFE **"));
    } else {
      sprintf(Buffer, "%2d", BurstSize);
      Oled.print(F("Burst: "));
      Oled.print(Buffer);
    }
  }

  byte DisplayDPS = 0;
  switch (TargetFireMode) {
    case FIRE_MODE_BURST:
      DisplayDPS = ROFAdjustmentB;
      break;
    case FIRE_MODE_AUTO:
      DisplayDPS = ROFAdjustmentA;
      break;
    default:
      DisplayDPS = 0;  // Single should show MAX
  }

  if (ClearScreen || (DisplayDPS != LastDisplayDPS)) {
    Oled.setFont(ZevvPeep8x16);
    Oled.setCursor(oled_adj + 0, 4);
    Oled.print(F("ROF: "));
    if (DisplayDPS == 0)
      Oled.print(F("MAX"));
    else {
      sprintf(Buffer, "%2d ", DisplayDPS);
      Oled.print(Buffer);
    }
  }

  if (ClearScreen || (MotorSpeedFull != LastMotorSpeedFull)) {
    Oled.setFont(ZevvPeep8x16);
    Oled.setCursor(oled_adj + 0, 6);
    Oled.print(F("Pwr:"));
    sprintf(Buffer, "%3d", MotorSpeedFull);
    Oled.print(Buffer);
    Oled.print(F("%"));
  }

  if (ClearScreen || (DartsInMag != LastDartsInMag)) {
    //Don't the screen if it's firing and the pusher is in the return stroke, unless the screen is refreshed.
    if (((UpdateAmmoCounter) || (!RunFiringSequence)) || (ClearScreen)) {
      LastRefresh = millis();
      Oled.setFont(ZevvPeep8x16);
      Oled.setCursor(oled_adj + 90, 3);
      Oled.set2X();
      sprintf(Buffer, "%2d", DartsInMag);
      Oled.print(Buffer);
      Oled.set1X();
      LastDartsInMag = DartsInMag;
      UpdateAmmoCounter = false;
    }
  }

  LastBurstSize = BurstSize;
  LastDisplayDPS = DisplayDPS;
  LastMotorSpeedFull = MotorSpeedFull;
  LastTargetFireMode = TargetFireMode;
}


// ------------------------------------------------------DISPLAY SCREEN HEADER ------------------------------
void Display_ScreenHeader(bool ClearScreen) {
  static float LastBatteryVoltage = 99.0;
  static byte LastCurrentProfile = 99;
  char Buffer[6];

  if (ClearScreen) {
    Oled.clear();
  }
  // Do not clear screen, voltage has changed
  if (ClearScreen || ((int)(LastBatteryVoltage * 10) != (int)(BatteryCurrentVoltage * 10))) {
    Oled.setFont(ZevvPeep8x16);
    Oled.setCursor(oled_adj + 0, 0);
    sprintf(Buffer, "%3d", (int)(BatteryCurrentVoltage * 10));
    Buffer[4] = 0;
    Buffer[3] = Buffer[2];
    Buffer[2] = '.';
    Oled.print(Buffer);
    Oled.print(F("v "));
  }

  if (ClearScreen || (LastCurrentProfile != CurrentProfile)) {
    Oled.setCursor(oled_adj + 70, 0);
    Oled.setFont(ZevvPeep8x16);
    Oled.print("Pro: ");
    Oled.print((char)('A' + CurrentProfile));
  }

  LastBatteryVoltage = BatteryCurrentVoltage;
  LastCurrentProfile = CurrentProfile;
}


//---------------------------------------------- DISPLAY LOW BATTERY ----------------------------------
void Display_LowBatt(bool ClearScreen) {
  if (ClearScreen) {
    Oled.setFont(ZevvPeep8x16);
    Oled.setCursor(oled_adj + 0, 2);
    Oled.print(F("################\n"));
    Oled.print(F("# LOW BATTERY! #\n"));
    Oled.print(F("################"));
  }
}

//-----------------------------------------------DISPLAY JAM SCREEN -----------------------
void Display_Jam(bool ClearScreen) {
  if (ClearScreen) {
    Oled.setFont(ZevvPeep8x16);
    Oled.setCursor(oled_adj + 0, 2);
    Oled.print(F("################\n"));
    Oled.print(F("# JAM DETECTED #\n"));
    Oled.print(F("################"));
  }
}


//
//----------------------------------- DIPSPLAY Config Screeen and write values to EEPROM ---------------
void Display_Config(bool ClearScreen) {
  static byte CurrentMenuItem = 0;
  static byte LastMenuItem = 99;
  static byte DisplayedProfile = 3;
  static bool InConfig = false;
  char Cursor;
  if (EnteringConfig) {
    CurrentMenuItem = 0;
    LastMenuItem = 99;
    UpdateMenuItems();
    InConfig = false;
  }
  if (DisplayedProfile != CurrentProfile) {
    DisplayedProfile = CurrentProfile;
    LastMenuItem = 99;
  }
  if (ClearScreen || (CurrentMenuItem != LastMenuItem)) {
    MenuItem* DisplayList[3];
    byte CursorItem = 0;
    if (CurrentMenuItem == 0) {
      DisplayList[0] = MenuItems[0];
      DisplayList[1] = MenuItems[1];
      DisplayList[2] = MenuItems[2];
      CursorItem = 0;
    } else if (CurrentMenuItem == (MENU_ITEMS - 1)) {
      DisplayList[0] = MenuItems[MENU_ITEMS - 3];
      DisplayList[1] = MenuItems[MENU_ITEMS - 2];
      DisplayList[2] = MenuItems[MENU_ITEMS - 1];
      CursorItem = 2;
    } else {
      DisplayList[0] = MenuItems[CurrentMenuItem - 1];
      DisplayList[1] = MenuItems[CurrentMenuItem];
      DisplayList[2] = MenuItems[CurrentMenuItem + 1];
      CursorItem = 1;
    }

    Oled.setFont(ZevvPeep8x16);

    if (InConfig)
      Cursor = '#';
    else
      Cursor = '>';

    Oled.setCursor(oled_adj + 0, 2);
    DisplayList[0]->PrepareOutput();
    if (CursorItem == 0)
      Oled.print(Cursor);
    else
      Oled.print(' ');
    Oled.print(DisplayList[0]->Title);
    Oled.print(' ');
    Oled.print(DisplayList[0]->Output);
    Oled.clearToEOL();

    Oled.setCursor(oled_adj + 0, 4);
    DisplayList[1]->PrepareOutput();
    if (CursorItem == 1)
      Oled.print(Cursor);
    else
      Oled.print(' ');
    Oled.print(DisplayList[1]->Title);
    Oled.print(' ');
    Oled.print(DisplayList[1]->Output);
    Oled.clearToEOL();

    Oled.setCursor(oled_adj + 0, 6);
    DisplayList[2]->PrepareOutput();
    if (CursorItem == 2)
      Oled.print(Cursor);
    else
      Oled.print(' ');
    Oled.print(DisplayList[2]->Title);
    Oled.print(' ');
    Oled.print(DisplayList[2]->Output);
    Oled.clearToEOL();

    LastMenuItem = CurrentMenuItem;
  }
  //=============================================================
  // Rotary here
  if (RotaryTurned == ROTARY_CW) {
    if (InConfig) {
      MenuItems[CurrentMenuItem]->Increase();
      LastMenuItem = 99;
    } else {
      if (CurrentMenuItem < (MENU_ITEMS - 1)) {
        CurrentMenuItem++;
      }
    }
  }
  if (RotaryTurned == ROTARY_CCW) {
    if (InConfig) {
      MenuItems[CurrentMenuItem]->Decrease();
      LastMenuItem = 99;
    } else {
      if (CurrentMenuItem > 0) {
        CurrentMenuItem--;
      }
    }
  }

  if ((ConsoleRotaryButtonAction == BTN_ACTION_SHORT) && !EnteringConfig) {
    switch (CurrentMenuItem) {
      case 0:  //exit request at begining of menu options
        SystemMode == SYSTEM_MODE_NORMAL;
        ExitingConfig = true;
        EnteringConfig = false;
        break;
      case 15:  //exit request at end of menu options
        SystemMode == SYSTEM_MODE_NORMAL;
        ExitingConfig = true;
        EnteringConfig = false;
        break;
      default:
        // Handle config stuff here
        InConfig = !InConfig;
        if (!InConfig) {
          switch (CurrentMenuItem) {
            case 1:
              MotorSpeedFull = MenuItems[1]->CurrentValue;
              Profiles[CurrentProfile].MotorSpeedFull = MotorSpeedFull;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_MSF, MotorSpeedFull);
              break;
            case 2:
              ROFAdjustmentA = MenuItems[2]->CurrentValue;
              Profiles[CurrentProfile].ROFAdjustmentA = ROFAdjustmentA;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_ROFA, ROFAdjustmentA);
              break;
            case 3:
              ROFAdjustmentB = MenuItems[3]->CurrentValue;
              Profiles[CurrentProfile].ROFAdjustmentB = ROFAdjustmentB;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_ROFB, ROFAdjustmentB);
              break;
            case 4:
              BurstSize = MenuItems[4]->CurrentValue;
              Profiles[CurrentProfile].BurstSize = BurstSize;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_BURST, BurstSize);
              break;
            case 5:
              MagSize = MenuItems[5]->CurrentValue;
              Profiles[CurrentProfile].MagSize = MagSize;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_MAGSIZE, MagSize);
              break;
            case 6:
              AccelerateTime = MenuItems[6]->CurrentValue;
              Profiles[CurrentProfile].AccelerateTime = AccelerateTime;
              EEPROM.put((CurrentProfile * ADDR_PROB_BASE) + ADDR_ACCEL, AccelerateTime);
              CalculateRampRates();  // Recalculate the ramp rates
              break;
            case 7:
              DecelerateTime = MenuItems[7]->CurrentValue;
              Profiles[CurrentProfile].DecelerateTime = DecelerateTime;
              EEPROM.put((CurrentProfile * ADDR_PROB_BASE) + ADDR_DECEL, DecelerateTime);
              CalculateRampRates();  // Recalculate the ramp rates
              break;
            case 8:
              MotorStartDwellTime = MenuItems[8]->CurrentValue;
              Profiles[CurrentProfile].MotorStartDwellTime = MotorStartDwellTime;
              EEPROM.put((CurrentProfile * ADDR_PROB_BASE) + ADDR_STARTD, MotorStartDwellTime);
              break;
            case 9:
              MotorStopDwellTime = MenuItems[9]->CurrentValue;
              Profiles[CurrentProfile].MotorStopDwellTime = MotorStopDwellTime;
              EEPROM.put((CurrentProfile * ADDR_PROB_BASE) + ADDR_STOPD, MotorStopDwellTime);
              break;
            case 10:
              PulseOnTimeHigh = MenuItems[10]->CurrentValue;
              Profiles[CurrentProfile].PulseOnTimeHigh = PulseOnTimeHigh;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_PULSE_HIGH, PulseOnTimeHigh);
              break;
            case 11:
              PulseOnTimeLow = MenuItems[11]->CurrentValue;
              Profiles[CurrentProfile].PulseOnTimeLow = PulseOnTimeLow;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_PULSE_LOW, PulseOnTimeLow);
              break;
            case 12:
              PulseRetractTime = MenuItems[12]->CurrentValue;
              Profiles[CurrentProfile].PulseRetractTime = PulseRetractTime;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_PULSE_RETRACT, PulseRetractTime);
              break;
            case 13:
              BatteyType = MenuItems[13]->CurrentValue;
              UpdateBatteryType();
              Profiles[CurrentProfile].BatteyType = BatteyType;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_BAT_TYPE, BatteyType);
              break;
            case 14:
              BatteryOffset = MenuItems[14]->CurrentValue;
              Profiles[CurrentProfile].BatteryOffset = BatteryOffset;
              EEPROM.write((CurrentProfile * ADDR_PROB_BASE) + ADDR_BAT_OFFSET, BatteryOffset);
              break;




              /*                      
 * Referance code
            case 10:
              ButtonActions[BTN_IDX_LS] = MenuItems[10]->CurrentValue;
              Profiles[CurrentProfile].BtnLS = ButtonActions[BTN_IDX_LS];
              EEPROM.write( (CurrentProfile * ADDR_PROB_BASE) + ADDR_BTN_LS, ButtonActions[BTN_IDX_LS] );
              break;
            case 11:
              ButtonActions[BTN_IDX_LL] = MenuItems[11]->CurrentValue;
              Profiles[CurrentProfile].BtnLL = ButtonActions[BTN_IDX_LL];
              EEPROM.write( (CurrentProfile * ADDR_PROB_BASE) + ADDR_BTN_LL, ButtonActions[BTN_IDX_LL] );
              break;
            case 12:
              ButtonActions[BTN_IDX_CS] = MenuItems[12]->CurrentValue;
              Profiles[CurrentProfile].BtnCS = ButtonActions[BTN_IDX_CS];
              EEPROM.write( (CurrentProfile * ADDR_PROB_BASE) + ADDR_BTN_CS, ButtonActions[BTN_IDX_CS] );
              break;              
            case 13:
              ButtonActions[BTN_IDX_CL] = MenuItems[13]->CurrentValue;
              Profiles[CurrentProfile].BtnCL = ButtonActions[BTN_IDX_CL];
              EEPROM.write( (CurrentProfile * ADDR_PROB_BASE) + ADDR_BTN_CL, ButtonActions[BTN_IDX_CL] );
              break;
            case 14:
              ButtonActions[BTN_IDX_RS] = MenuItems[14]->CurrentValue;
              Profiles[CurrentProfile].BtnRS = ButtonActions[BTN_IDX_RS];
              EEPROM.write( (CurrentProfile * ADDR_PROB_BASE) + ADDR_BTN_RS, ButtonActions[BTN_IDX_RS] );
              break;
            case 15:
              ButtonActions[BTN_IDX_RL] = MenuItems[15]->CurrentValue;
              Profiles[CurrentProfile].BtnRL = ButtonActions[BTN_IDX_RL];
              EEPROM.write( (CurrentProfile * ADDR_PROB_BASE) + ADDR_BTN_RL, ButtonActions[BTN_IDX_RL] );
              break;
            case 16: default:
              ButtonActions[BTN_IDX_ROT] = MenuItems[16]->CurrentValue;
              Profiles[CurrentProfile].BtnRot = ButtonActions[BTN_IDX_ROT];
              EEPROM.write( (CurrentProfile * ADDR_PROB_BASE) + ADDR_BTN_ROT, ButtonActions[BTN_IDX_ROT] );
              break; 
              */
          }
        }
        LastMenuItem = 99;
        break;
    }
  }

  if (EnteringConfig)
    EnteringConfig = false;
}

// --------------------------------------------------------If battery Type is changed these constantes need to change-------------------
void UpdateBatteryType() {

  if (BatteyType == BATTERY_3S) {
    BatteryMinVoltage = BATTERY_3S_MIN;
    BatteryMaxVoltage = BATTERY_3S_MAX;
    SolenoidLowVoltage = (int)(BATTERY_3S_MIN * 10.0);
    SolenoidHighVoltage = (int)(BATTERY_3S_MAX * 10.0);
  }
  if (BatteyType == BATTERY_4S) {
    BatteryMinVoltage = BATTERY_4S_MIN;
    BatteryMaxVoltage = BATTERY_4S_MAX;
    SolenoidLowVoltage = (int)(BATTERY_4S_MIN * 10.0);
    SolenoidHighVoltage = (int)(BATTERY_4S_MAX * 10.0);
  }
  //  Serial.println ( BatteyType );
  //  Serial.println ( BatteryMinVoltage );
  //  Serial.println ( BatteryMaxVoltage );
  //  Serial.println ( SolenoidLowVoltage );
  //  Serial.println ( SolenoidHighVoltage );
}

// ---------------------------------------------------------------------------
void UpdateMenuItems() {
  // Ignore [0]
  MenuItems[1]->CurrentValue = MotorSpeedFull;
  MenuItems[2]->CurrentValue = ROFAdjustmentA;
  MenuItems[3]->CurrentValue = ROFAdjustmentB;
  MenuItems[4]->CurrentValue = BurstSize;
  MenuItems[5]->CurrentValue = MagSize;
  MenuItems[6]->CurrentValue = AccelerateTime;       //ramp U
  MenuItems[7]->CurrentValue = DecelerateTime;       //Ramp D
  MenuItems[8]->CurrentValue = MotorStartDwellTime;  //Dwel U
  MenuItems[9]->CurrentValue = MotorStopDwellTime;   //Dwel D
  MenuItems[10]->CurrentValue = PulseOnTimeHigh;
  MenuItems[11]->CurrentValue = PulseOnTimeLow;
  MenuItems[12]->CurrentValue = PulseRetractTime;
  MenuItems[13]->CurrentValue = BatteyType;
  MenuItems[14]->CurrentValue = BatteryOffset;
  // MenuItem [15} is EXIT

  // int num =atoi(200);
  // --- Referance code ---
  //MenuItems[10]->CurrentValue = ButtonActions[BTN_IDX_LS];
  //MenuItems[11]->CurrentValue = ButtonActions[BTN_IDX_LL];
  //MenuItems[12]->CurrentValue = ButtonActions[BTN_IDX_CS];
  //MenuItems[13]->CurrentValue = ButtonActions[BTN_IDX_CL];
  //MenuItems[14]->CurrentValue = ButtonActions[BTN_IDX_RS];
  //MenuItems[15]->CurrentValue = ButtonActions[BTN_IDX_RL];
  //MenuItems[16]->CurrentValue = ButtonActions[BTN_IDX_ROT];
}

//--------------------------------------------------------------- only called at start up -----------------------------------
void UpdateProfile() {
  MotorSpeedFull = Profiles[CurrentProfile].MotorSpeedFull;
  ROFAdjustmentA = Profiles[CurrentProfile].ROFAdjustmentA;
  ROFAdjustmentB = Profiles[CurrentProfile].ROFAdjustmentB;
  BurstSize = Profiles[CurrentProfile].BurstSize;
  MagSize = Profiles[CurrentProfile].MagSize;
  AccelerateTime = Profiles[CurrentProfile].AccelerateTime;
  DecelerateTime = Profiles[CurrentProfile].DecelerateTime;
  MotorStartDwellTime = Profiles[CurrentProfile].MotorStartDwellTime;
  MotorStopDwellTime = Profiles[CurrentProfile].MotorStopDwellTime;
  PulseOnTimeHigh = Profiles[CurrentProfile].PulseOnTimeHigh;
  PulseOnTimeLow = Profiles[CurrentProfile].PulseOnTimeLow;
  PulseRetractTime = Profiles[CurrentProfile].PulseRetractTime;
  BatteyType = Profiles[CurrentProfile].BatteyType;
  BatteryOffset = Profiles[CurrentProfile].BatteryOffset;


  // --- Referance code ---
  //ButtonActions[BTN_IDX_LS] = Profiles[CurrentProfile].BtnLS;
  //ButtonActions[BTN_IDX_LL] = Profiles[CurrentProfile].BtnLL;
  //ButtonActions[BTN_IDX_CS] = Profiles[CurrentProfile].BtnCS;
  //ButtonActions[BTN_IDX_CL] = Profiles[CurrentProfile].BtnCL;
  //ButtonActions[BTN_IDX_RS] = Profiles[CurrentProfile].BtnRS;
  //ButtonActions[BTN_IDX_RL] = Profiles[CurrentProfile].BtnRL;
  //ButtonActions[BTN_IDX_ROT] = Profiles[CurrentProfile].BtnRot;

  CalculateRampRates();  // Recalculate the ramp rates
  UpdateBatteryType();
}
