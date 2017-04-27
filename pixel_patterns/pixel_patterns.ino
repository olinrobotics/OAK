//#include "NeoPatterns.h"
#include "Emotion.h"
#include <inttypes.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      12

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// For RGB_W Neopixel rings, the third parameter needs to be changed as so. Examples of alternative paramenters
// for other versions, see stradtest example provided with the Adafruit library

//Adafruit_NeoPixel ring = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);
void RingComplete();
void RingComplete2();

// Make it NEO_GRBW because the red and green are switched on these neopixels for some reason.
// By setting this value, it can be treated as RGB
//NeoPatterns Ring(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800, NULL);
Emotion e(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);
// Initialize everything and prepare to start
void setup()
{
  Serial.begin(115200);
  e.Praise();
}

// Main loop
void loop()
{
  // Update the ring
  e.Update();
  // Any interrupts or updates to patterns go here, like button presses
}
