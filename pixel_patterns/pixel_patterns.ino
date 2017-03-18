#include "NeoPatterns.h"
#include <inttypes.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      12

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// For RGB_W Neopixel rings, the third parameter needs to be changed as so. Examples of alternative paramenters
// for other versions, see stradtest example provided with the Adafruit library

//Adafruit_NeoPixel ring = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);
void RingComplete();

NeoPatterns Ring(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800, NULL);

// Initialize everything and prepare to start
void setup()
{
  Serial.begin(115200);

  // Initialize all the pixelStrips
  Ring.begin();

  // Kick off a pattern
  //Ring.TheaterChase(Ring.Color(255, 255, 0), Ring.Color(0, 0, 50), 100);
  //Ring.RainbowCycle(30);
  Ring.Breathe(Ring.Color(165, 165, 14), 60, 40);
}

// Main loop
void loop()
{
  // Update the ring
  Ring.Update();
  // Any interrupts or updates to patterns go here, like button presses
}

//------------------------------------------------------------
// Completion Routines - get called on completion of a pattern
//------------------------------------------------------------

// Ring Completion Callback
void RingComplete()
{
  //Ring.RainbowCycle(10);
//  long randomint = random(300);
//  if (randomint % 300 == 0) // If random number is divisible by 4
//  {
//    Ring.TheaterChase(Ring.Color(255, 255, 0), Ring.Color(0, 0, 50), 100);
////    // Speed up Ring1 and change to Fade
//  }
//  else  // Return to normal
//  {
//    Ring.ActivePattern = RAINBOW_CYCLE;
//    //Ring.Interval = 10;
//  }
}
