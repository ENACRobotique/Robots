#ifndef _SPRIT_H
#define _SPRIT_H

#include "Arduino.h"
#include "Adafruit_GFX.h"
#include "RGBmatrixPanel.h"

#include <SPI.h>

#define CLK 8  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT A4
#define OE  9
#define A   A0
#define B   A1
#define C   A2
#define D   A3


void init_matrix();

void erase_all();

void drawPixel2(int16_t x, int16_t y, uint8_t r,uint8_t g,uint8_t b,int8_t offset);
inline void drawPixel2(int16_t x, int16_t y, uint8_t r,uint8_t g,uint8_t b){
  return drawPixel2(x,y,r,g,b,0);
}


#endif _SPRIT_H
