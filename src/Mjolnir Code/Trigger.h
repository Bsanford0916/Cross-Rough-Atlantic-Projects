#include "Settings.h"

// Pin macros
/*
 * Read pin state
 * 
 * Valid for:
 * PC2/D16 - Trigger                           FIRE_T
 * 
 */

#define GET_FIRE_T ((PINC & 0b00000100) ? HIGH : LOW)  // PC2 this is what we want

byte TriggerButtonState = BTN_HIGH;

volatile bool TriggerChanged = false;  // ISR use

volatile byte LastPINC = 0;  // Preload with PINC Prior to events happening

// ---------------------------------------------------------------------------------------
ISR(PCINT1_vect)  // Trigger - PORT C PCINT - PC2
{
  // Trigger is PC2
  if ((PINC & 0b00000100) != (LastPINC & 0b00000100)) {
    TriggerChanged = true;
  }

  LastPINC = PINC;
}