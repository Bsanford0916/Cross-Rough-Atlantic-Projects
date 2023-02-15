#ifndef OLED_H
#define OLED_H

#include "Logo.h"

void InitDisplay(){
 // Boot LCD
  Serial.println(F("Initialising Display"));
  Wire.begin();
  Wire.setClock(400000L);                    // 400khz
  Oled.begin(&Adafruit128x64, OLED_ADDR);    // 0.96 Inch display
  Serial.println(F("Display Initialised"));  //

  // Give a nice message
  Oled.clear();
  Oled.displayRemap(false);  // set true or false to flip screen
  Oled.setFont(ZevvPeep8x16);
  Oled.setCursor(oled_adj + 40, 2);
  Oled.print(F("BOO"));
  Oled.setCursor(oled_adj + 35, 4);
  Oled.print(F("Booting"));
}

  // Display logo                                      
  // The pic lives in Logo.h.
void DisplayLogo() {
  Oled.clear();
  for (byte Row = 0; Row < 8; Row++) {
    Wire.beginTransmission(OLED_ADDR);  // Set the memory page
    Wire.write(0x00);
    Wire.write(0xB0 | Row & 7);
    Wire.endTransmission();
    Wire.beginTransmission(OLED_ADDR);  // Dump data 16 bytes per i2c packet
    Wire.write(0x40);
    int Seg = 0;
    for (byte Col = 0; Col < oled_cols; Col++) {
      Seg++;
      if (Seg == 16) {
        Wire.endTransmission();
        Wire.beginTransmission(OLED_ADDR);
        Wire.write(0x40);
        Seg = 0;
      }
      Wire.write(~(pgm_read_byte_near(Logo + Row * 128 + Col)));
    }
    Wire.endTransmission();
  }
}

#endif