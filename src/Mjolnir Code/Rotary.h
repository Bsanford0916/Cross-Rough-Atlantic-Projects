#ifndef ROTARY_H
#define ROTARY_H

#include "Settings.h"
// Pin macros
/*
 * Read pin state
 * 
 * Valid for:
 * PB4/D12 - Encoder Sw                        ENCODER_SW
 * PD2/INT0/D2 - Encoder CLK                   ENCODER_CLK
 * PD3/INT1/D3 - Encoder DT                    ENCODER DT
 */

#define GET_ENCODER_SW ((PINB & 0b00010000) ? HIGH : LOW)   // PB4
#define GET_ENCODER_CLK ((PIND & 0b00001000) ? HIGH : LOW)  // PD3 This is encoder Clk
#define GET_ENCODER_DT ((PIND & 0b00000100) ? HIGH : LOW)   // PD2 This is encoder DT

//------------------------------------------------------------------------------
// Rotary Encoder Variables

volatile byte aFlag = 0;       // let's us know when we're expecting a rising edge on PIN_ENCODER_CK to signal that the encoder has arrived at a detent
volatile byte bFlag = 0;       // let's us know when we're expecting a rising edge on PIN_ENCODER_DT to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0;  // this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0;   // stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0;     // somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

volatile bool EncoderSwChanged = false;  // ISR use

byte ConsoleRotaryAButtonState = BTN_LOW;  //Encoder DT or CLK
byte ConsoleRotaryBButtonState = BTN_LOW;  //ENCODER DT or CLK
byte ConsoleRotaryCButtonState = BTN_LOW;  //ENcoder Sw

#define ROTARY_NONE 0
#define ROTARY_CW 1
#define ROTARY_CCW 2
byte RotaryTurned = ROTARY_NONE;

byte ConsoleRotaryButtonAction = BTN_ACTION_NONE;  //encoder rotary button

volatile byte LastPINB = 0;  // Preload with PINB Prior to events happening


//======================================== Interupt Servce Routines =========================================================================
ISR(INT0_vect)  //Rotary Encoder CLK D2 INT0 (hardware Interupt)
{
  cli();                                // stop interrupts happening before we read pin values
  reading = PIND & 0xC;                 // read all eight pin values then strip away all but PIN_ENCODER_CK and PIN_ENCODER_DT's values
  if (reading == B00001100 && aFlag) {  // check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos--;                       //decrement the encoder's position count. CAN BE DELETED
    //RotaryTurned = ROTARY_CCW;
    bFlag = 0;                                 //reset flags for the next turn
    aFlag = 0;                                 //reset flags for the next turn
  } else if (reading == B00000100) bFlag = 1;  //signal that we're expecting PIN_ENCODER_DT to signal the transition to detent from free rotation
  sei();                                       //restart interrupts
}

ISR(INT1_vect)  //Rotary Encoder DT D2 INT1 (Hardware Interupt)
{
  cli();                                //stop interrupts happening before we read pin values
  reading = PIND & 0xC;                 //read all eight pin values then strip away all but PIN_ENCODER_CK and PIN_ENCODER_DT's values
  if (reading == B00001100 && bFlag) {  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos++;                       //increment the encoder's position count CAN BE DETELTED
    //RotaryTurned = ROTARY_CW;
    bFlag = 0;                                 //reset flags for the next turn
    aFlag = 0;                                 //reset flags for the next turn
  } else if (reading == B00001000) aFlag = 1;  //signal that we're expecting PIN_ENCODER_CK to signal the transition to detent from free rotation
  sei();                                       //restart interrupts
}


#endif
