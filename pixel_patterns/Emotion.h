#include <Adafruit_NeoPixel.h>
#include "NeoPatterns.h"

class Emotion{
  private:
    NeoPatterns Ring;
//    // Which pin on the Arduino is connected to the NeoPixels?
//    #define PIN            6
//    
//    // How many NeoPixels are attached to the Arduino?
//    #define NUMPIXELS      12
//
//    NeoPatterns Ring(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800, NULL);
    static const short MAX_STEPS = 10;
    static const short NUM_COLOR_PARAMS = 2;
    static const short NUM_TIME_PARAMS = 2;
        
    short Functions[MAX_STEPS]; // List of numbers mapped to functions
    uint32_t ColorParams[MAX_STEPS][NUM_COLOR_PARAMS]; // List of color pairs
    uint8_t TimeParams[MAX_STEPS][NUM_TIME_PARAMS]; // List of (# steps, interval) pairs
    int Durations[MAX_STEPS]; // Total duration of each function
    int Index;
    int TotalSteps; // <= MAX_STEPS
    unsigned long lastUpdate;
    
    public:
    // Constructor: Initialize Ring
    Emotion(uint16_t pixels, uint8_t pin, uint8_t type) : Ring(pixels, pin, type, NULL)
    {
      Ring.begin();
    }
    
    void Update(){
      // Check if it is time to switch color functions, then update the ring which will run the color function at its discretion
      if ((millis() - lastUpdate) > Durations[Index]){ // time to update
        lastUpdate = millis();
        // Update function
        if(Index < TotalSteps - 1){ // Update the function, otherwise, stay at the same index
          Index++;
          UpdatePattern();
        }
      }
      Ring.Update();
    }
    // Update the pattern in steps or idle
    void UpdatePattern(){
      // update the pattern that is currently being run by launching the Ring with the correct params
      // Then, update the ring.
      switch (Functions[Index]){
          case 1:
            Ring.RainbowCycle(TimeParams[Index][1]);
            break;
          case 2:
            Ring.TheaterChase(ColorParams[Index][0], ColorParams[Index][1], TimeParams[Index][1]);
            break;
          case 3:
            Ring.ColorWipe(ColorParams[Index][0], TimeParams[Index][1]);
            break;
          case 4:
            Ring.Scanner(ColorParams[Index][0], TimeParams[Index][1]);
            break;
          case 5:
            Ring.Pulse(ColorParams[Index][0], ColorParams[Index][1], TimeParams[Index][0], TimeParams[Index][1]);
            break;
          case 6:
            Ring.Fade(ColorParams[Index][0], ColorParams[Index][1], TimeParams[Index][0], TimeParams[Index][1]);
            break;
          case 7:
            Ring.NoColor(TimeParams[Index][0], TimeParams[Index][1]);
            break;
          default:
            break;
        }
    }

    void Praise(){
      // Quick circle of orange turning to a slow pulse between yellow and orange
        uint32_t yellow = Ring.Color(239, 194, 14);
        uint32_t orange = Ring.Color(239, 127, 14);
        Index = 0;
        TotalSteps = 2; // Number of functions in the pattern

        short PraiseFunctions[TotalSteps] = {3, 5};
        uint32_t PraiseColorParams[TotalSteps][NUM_COLOR_PARAMS] = {{orange, 0},{orange, yellow}};
        uint8_t PraiseTimeParams[TotalSteps][NUM_TIME_PARAMS] = {{0, 50},{20, 40}};
        
        updateArrays(PraiseFunctions, PraiseColorParams, PraiseTimeParams);
        CalculateFunctionDurations();
        UpdatePattern();
    }

    void Stop(){
      Index = 0;
      TotalSteps = 1;
      Ring.NoColor(10, 10);
    }

    void CalculateFunctionDurations(){
      for(int i = 0; i < MAX_STEPS; i++){
        // Define special function durations which have not standard step sizes defined in NeoPatterns.
        if(i < TotalSteps){
          switch(Functions[i]){
            case 1: // RainbowCycle
              Durations[i] = 255 * TimeParams[i][1];
              break;
            case 4: // Scanner
              Durations[i] = (Ring.numPixels() - 1) * 2 * TimeParams[i][1];
              break;
            case 5: // Pulse
              Durations[i] = 2 * TimeParams[i][0] * TimeParams[i][1];
//              Serial.println(Durations[i]);
              break;
            default:
              if(TimeParams[i][0] == 0){
                if(TimeParams[i][1] == 0){
                  Durations[i] = 1; // One step
                } else{
                  Durations[i] = Ring.numPixels() * TimeParams[i][1]; // Steps = number of pixels
                }
              } else{
                Durations[i] = TimeParams[i][0] * TimeParams[i][1]; // Steps is not modifed when activating pattern in ring
              }
              break;
          }
        } else{
          Durations[i] = 0;
        }
          Serial.print(Durations[i]);
          Serial.print(", ");
          
      }
      Serial.println();
    }

  void updateArrays(short newFunctions[], uint32_t newColorParams[][NUM_COLOR_PARAMS], uint8_t newTimeParams[][NUM_COLOR_PARAMS]){
    for(int i = 0; i < MAX_STEPS; i++){
      if(i < TotalSteps){ // Total steps should be updated before calling this function
        Functions[i] = newFunctions[i];
        for(int j = 0; j < NUM_COLOR_PARAMS; j++){
          ColorParams[i][j] = newColorParams[i][j];
          TimeParams[i][j] = newTimeParams[i][j];
        }
        
//        for(int j = 0; j < NUM_TIME_PARAMS; j++){
//          TimeParams[i][j] = newTimeParams[i][j];
//        }
      }else{
        Functions[i] = 0;
        for(int j = 0; j < NUM_COLOR_PARAMS; j++){
          ColorParams[i][j] = 0;
          TimeParams[i][j] = 0;
        }
        
//        for(int j = 0; j < NUM_TIME_PARAMS; j++){
//          TimeParams[i][j] = 0;
//        }
      }
//      Serial.print("{");
//      Serial.print(TimeParams[i][0]);
//      Serial.print(", ");
//      Serial.print(TimeParams[i][1]);
//      Serial.print("}, ");
    }
//    Serial.println();
  }

};
