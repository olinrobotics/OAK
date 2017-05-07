#include <Adafruit_NeoPixel.h>
#include "NeoPatterns.h"

class Emotion{
  private:
    NeoPatterns Ring;
    static const short MAX_STEPS = 10;
    static const short NUM_COLOR_PARAMS = 2;
    static const short NUM_TIME_PARAMS = 2;
        
    short Functions[MAX_STEPS]; // List of numbers mapped to functions
    uint32_t ColorParams[MAX_STEPS][NUM_COLOR_PARAMS]; // List of color pairs
    uint8_t TimeParams[MAX_STEPS][NUM_TIME_PARAMS]; // List of (# steps, interval) pairs
    short Durations[MAX_STEPS]; // Total duration of each function
    short Index;
    short TotalSteps; // <= MAX_STEPS
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
            Ring.Cycle(ColorParams[Index][0], TimeParams[Index][1]);
            break;
          case 5:
            Ring.Wiper(ColorParams[Index][0], TimeParams[Index][1]);
            break;
          case 6:
            Ring.Pulse(ColorParams[Index][0], ColorParams[Index][1], TimeParams[Index][0], TimeParams[Index][1]);
            break;
          case 7:
            Ring.Fade(ColorParams[Index][0], ColorParams[Index][1], TimeParams[Index][0], TimeParams[Index][1]);
            break;
          case 8:
            Ring.NoColor(TimeParams[Index][0], TimeParams[Index][1]);
            break;
          default:
            break;
        }
    }

    void Praise(){
      // Quick circle of orange turning to a slow pulse between yellow and orange
        uint32_t yellow = Ring.Color(239, 164, 14);
        uint32_t orange = Ring.Color(239, 119, 14);
        TotalSteps = 2; // Number of functions in the pattern

        short PraiseFunctions[TotalSteps] = {3, 6};
        uint32_t PraiseColorParams[TotalSteps][NUM_COLOR_PARAMS] = {{orange, 0},{orange, yellow}};
        uint8_t PraiseTimeParams[TotalSteps][NUM_TIME_PARAMS] = {{0, 50},{20, 60}};
        
        UpdateNewEmotion(PraiseFunctions, PraiseColorParams, PraiseTimeParams);
    }

    void Gloat(){
      // Quick cycle in blue, quick cycle in green, super quick pulsing
        uint32_t blue = Ring.Color(39, 64, 229);
        uint32_t brightGreen = Ring.Color(34, 232, 57);
        TotalSteps = 3; // Number of functions in the pattern

        short GloatFunctions[TotalSteps] = {4, 4, 6};
        uint32_t GloatColorParams[TotalSteps][NUM_COLOR_PARAMS] = {{blue, 0}, {brightGreen, 0}, {blue, brightGreen}};
        uint8_t GloatTimeParams[TotalSteps][NUM_TIME_PARAMS] = {{0, 20}, {0, 20}, {5, 30}};
        UpdateNewEmotion(GloatFunctions, GloatColorParams, GloatTimeParams);
    }

    void Blush(){
      // Slow fade into pinkish color and pulse very slightly
      uint32_t no_color = Ring.Color(0, 0, 0);
      uint32_t light_pink = Ring.Color(224, 65, 72);
      uint32_t dark_pink = Ring.Color(224, 72, 65);
      TotalSteps = 2;
      
      short BlushFunctions[TotalSteps] = {7, 6};
      uint32_t BlushColorParams[TotalSteps][NUM_COLOR_PARAMS] = {{no_color, light_pink}, {light_pink, dark_pink}};
      uint8_t BlushTimeParams[TotalSteps][NUM_TIME_PARAMS] = {{50, 50}, {50, 100}};
      
      UpdateNewEmotion(BlushFunctions, BlushColorParams, BlushTimeParams);
    }
    void NoEmotion(){
      // Turn off the NeoPixels
      TotalSteps = 1;
      short NoEmotionFunctions[TotalSteps] = {8};
      uint32_t NoEmotionColorParams[TotalSteps][NUM_COLOR_PARAMS] = {{0, 0}};
      uint8_t NoEmotionTimeParams[TotalSteps][NUM_TIME_PARAMS] = {{10, 10}};
      
      UpdateNewEmotion(NoEmotionFunctions, NoEmotionColorParams, NoEmotionTimeParams);
    }

    void TestEmotion(){
      uint32_t purple = Ring.Color(125, 22, 204);
      TotalSteps = 1;
      short TestFunctions[TotalSteps] = {4};
      uint32_t TestColorParams[TotalSteps][NUM_COLOR_PARAMS] = {{purple, 0}};
      uint8_t TestTimeParams[TotalSteps][NUM_TIME_PARAMS] = {{0, 100}};
      
      UpdateNewEmotion(TestFunctions, TestColorParams, TestTimeParams);
    }

    // To start new emotion sequence, update the global functions and their corresponding parameter arrays, calculate how much
    // time each step in the sequence takes, and set the pattern on the Neopixel.
    void UpdateNewEmotion(short NewFunctions[], uint32_t NewColorParams[][NUM_COLOR_PARAMS], uint8_t NewTimeParams[][NUM_COLOR_PARAMS]){
      Index = 0; // Start from the beginning of the array of new functions
      UpdateArrays(NewFunctions, NewColorParams, NewTimeParams);
      CalculateFunctionDurations();
      UpdatePattern();
    }


  // Set the global functions and their associated parameter arrays to the new emotion sequences. Because in C you cannot
  // replace an array with a new one explicitly, you need to loop through every element (which is a pointer to a memory location) 
  // and update the value in that block of memory. Arrays also cannot change size, so whatever space you do not use in the array
  // set the value to 0. For example, if the maximum steps you set is 10 and your emotion needs only 3 steps, the first 3 elements
  // are set to the values you provide, while the other 7 elements should be 0.
  void UpdateArrays(short NewFunctions[], uint32_t NewColorParams[][NUM_COLOR_PARAMS], uint8_t NewTimeParams[][NUM_COLOR_PARAMS]){
    for(int i = 0; i < MAX_STEPS; i++){
      if(i < TotalSteps){ // Total steps in the emotion sequence should be updated before calling this function
        Functions[i] = NewFunctions[i];
        // Because NUM_COLOR_PARAMS and NUM_TIME_PARAMS is the same now, they can be looped through together.
        // If this becomes not the case, you will need to separate loops.
        for(int j = 0; j < NUM_COLOR_PARAMS; j++){
          ColorParams[i][j] = NewColorParams[i][j];
          TimeParams[i][j] = NewTimeParams[i][j];
        }
        
      }else{
        Functions[i] = 0;
        for(int j = 0; j < NUM_COLOR_PARAMS; j++){
          ColorParams[i][j] = 0;
          TimeParams[i][j] = 0;
        }
      }
    }
  }

    // Calculate how much time each step in the emotion sequence needs to run and update the durations array.
    // Switching between steps of a sequence is determined by the duration of each step.
    void CalculateFunctionDurations(){
      for(int i = 0; i < MAX_STEPS; i++){
        // Define special function durations which have not standard step sizes defined in NeoPatterns.
        if(i < TotalSteps){
          switch(Functions[i]){
            case 1: // RainbowCycle
              Durations[i] = 255 * TimeParams[i][1];
              break;
            case 5: // Wiper
              Durations[i] = (Ring.numPixels() - 1) * 2 * TimeParams[i][1];
              break;
            case 6: // Pulse
              Durations[i] = 2 * TimeParams[i][0] * TimeParams[i][1];
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
          Durations[i] = 0; // other indeces are not taken up by steps of the emotion sequence
        }
//          Serial.print(Durations[i]);
//          Serial.print(", ");
          
      }
//      Serial.println();
    }

  
};
