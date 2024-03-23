#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>


// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        16 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 1 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void showColor(uint8_t r, uint8_t g, uint8_t b, int time = 1){
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
  delay(time);
}

#endif