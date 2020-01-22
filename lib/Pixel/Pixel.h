#ifndef PIXEL_H
#define PIXEL_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

void colorWipe(Adafruit_NeoPixel strip, uint32_t color, int wait);
void rainbow(Adafruit_NeoPixel strip, int wait, int speed, int brightness);

#endif