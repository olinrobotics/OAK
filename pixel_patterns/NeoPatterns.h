#include <Adafruit_NeoPixel.h>

// Pattern types supported:
enum  pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, COLOR_WIPE, CYCLE, FADE, NO_COLOR, PULSE, WIPER };
// Patern directions supported:
enum  direction { FORWARD, REVERSE };

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel{
  public:
    // Member Variables:
    pattern  ActivePattern;  // which pattern is running
    direction Direction;     // direction to run the pattern

    unsigned long Interval;   // milliseconds between updates
    unsigned long lastUpdate; // last update of position
    
    uint32_t Color1, Color2;  // What colors are in use
    uint16_t TotalSteps;  // total number of steps in the pattern
    int16_t Index;  // current step within the pattern

    void (*OnComplete)();  // Callback on completion of pattern

    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
      : Adafruit_NeoPixel(pixels, pin, type)
    {
      OnComplete = callback;
    }

    // Update the pattern
    void Update(){
      if ((millis() - lastUpdate) > Interval){ // time to update
        lastUpdate = millis();
        switch (ActivePattern){
          case RAINBOW_CYCLE:
            RainbowCycleUpdate();
            break;
          case THEATER_CHASE:
            TheaterChaseUpdate();
            break;
          case COLOR_WIPE:
            ColorWipeUpdate();
            break;
          case CYCLE:
            CycleUpdate();
            break;
          case WIPER:
            WiperUpdate();
            break;
          case PULSE:
            PulseUpdate();
            break;
          case FADE:
            FadeUpdate();
            break;
          case NO_COLOR:
            NoColorUpdate();
            break;
          default:
            break;
        }
      }
    }
    // Increment the Index and reset at the end
    void Increment(){
    
      if (Direction == FORWARD){
        Index++;
        if (Index >= TotalSteps){
          Index = 0;
          if (OnComplete != NULL){
            OnComplete(); // call the completion callback
          }
        }
      }
      else{ // Direction == REVERSE
        --Index;
        if (Index < 0){
          Index = TotalSteps - 1;
          if (OnComplete != NULL){
            OnComplete(); // call the completion callback
          }
        }
      }
    }
    // Reverse pattern direction
    void Reverse(){
      if (Direction == FORWARD){
        Direction = REVERSE;
        Index = TotalSteps - 1;
      }
      else{
        Direction = FORWARD;
        Index = 0;
      }
    }

    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval, direction dir = FORWARD){
      ActivePattern = RAINBOW_CYCLE;
      Interval = interval;
      TotalSteps = 255;
      Index = 0;
      Direction = dir;
    }

    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate(){
      for (int i = 0; i < numPixels(); i++){
        setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
      }
      uint32_t c = Wheel(((1 * 256 / numPixels()) + Index) & 255);
      show();
      Increment();
    }

    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD){
      ActivePattern = THEATER_CHASE;
      Interval = interval;
      TotalSteps = numPixels();
      Color1 = color1;
      Color2 = color2;
      Index = 0;
      Direction = dir;
    }

    // Update the Theater Chase Pattern
    void TheaterChaseUpdate(){
      for (int i = 0; i < numPixels(); i++){
        if ((i + Index) % 3 == 0){
          setPixelColor(i, Color1);
        }
        else{
          setPixelColor(i, Color2);
        }
      }
      show();
      Increment();
    }

    // Color Wipe paints a color, one pixel at a time, over the length of the strip resulting in all pixels of that color
    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD){
      ActivePattern = COLOR_WIPE;
      Interval = interval;
      TotalSteps = numPixels();
      Color1 = color;
      Index = (dir == FORWARD) ? (0) : (TotalSteps - 1);
      Direction = dir;
//      ColorSet(Color(0, 0, 0));
      show();
    }

    // Update the Color Wipe Pattern
    void ColorWipeUpdate(){
      setPixelColor(Index, Color1);
      show();
      Increment();
    }

    // Cycle moves a bright dot with a tail of dimming lights along the circle.
    // Initialize for a CYCLE
    void Cycle(uint32_t color1, uint8_t interval){
      ActivePattern = CYCLE;
      Interval = interval;
      TotalSteps = numPixels();
      Color1 = color1;
      Index = 0;
    }

    // Update the Cycle Pattern
    void CycleUpdate(){
      for (int i = 0; i < numPixels(); i++){
        if (i == Index){ // Scan Pixel to the right
          setPixelColor(i, Color1);
        }
        else{ // Fading tail
          setPixelColor(i, DimColor(getPixelColor(i)));
        }
      }
      show();
      Increment();
    }

    // Wiper moves a bright dot of color with a tail of dimming lights back and forth along the circle.
    // Note: This is defined as the Scanner function in the Adafruit library
    // Initialize for a WIPER
    void Wiper(uint32_t color1, uint8_t interval){
      ActivePattern = WIPER;
      Interval = interval;
      TotalSteps = (numPixels() - 1) * 2;
      Color1 = color1;
      Index = 0;
    }

    // Update the Wiper Pattern
    void WiperUpdate(){
      for (int i = 0; i < numPixels(); i++){
        if (i == Index){ // Scan Pixel to the right
          setPixelColor(i, Color1);
        }
        else if (i == TotalSteps - Index){ // Scan Pixel to the left
          setPixelColor(i, Color1);
        }
        else{ // Fading tail
          setPixelColor(i, DimColor(getPixelColor(i)));
        }
      }
      show();
      Increment();
    }

    // Pulse fades from one color to another and back continuously
    void Pulse(uint32_t color1, uint32_t color2, uint16_t steps, uint8_t interval){
      ActivePattern = PULSE;
      Interval = interval;
      TotalSteps = steps * 2;
      Color1 = color1;
      Color2 = color2;
      Index = 0;
      Direction = FORWARD;
    }

    // Update the Pulse Pattern
    void PulseUpdate(){
      // Calculate linear interpolation between Color1 and Color2
      // Optimise order of operations to minimize truncation error
      int total_half_steps = TotalSteps / 2;
      int index_half = Index;
      if(Index > total_half_steps){
        index_half = TotalSteps - Index;
      }
//      Serial.println(index_half);
      uint8_t red = ((Red(Color1) * (total_half_steps - index_half)) + (Red(Color2) * index_half)) / total_half_steps;
      uint8_t green = ((Green(Color1) * (total_half_steps - index_half)) + (Green(Color2) * index_half)) / total_half_steps;
      uint8_t blue = ((Blue(Color1) * (total_half_steps - index_half)) + (Blue(Color2) * index_half)) / total_half_steps;

      ColorSet(Color(red, green, blue));
      show();
      Increment();
    }

    

    // Initialize for a Fade
    void Fade(uint32_t color1, uint32_t color2, uint16_t steps, uint8_t interval, direction dir = FORWARD){
      ActivePattern = FADE;
      Interval = interval;
      TotalSteps = steps;
      Color1 = color1;
      Color2 = color2;
      Index = 0;
      Direction = dir;
    }

    // Update the Fade Pattern
    void FadeUpdate(){
      // Calculate linear interpolation between Color1 and Color2
      // Optimise order of operations to minimize truncation error
      uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
      uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
      uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;

      ColorSet(Color(red, green, blue));
      show();
      Increment();
    }

    void NoColor(uint16_t steps, uint8_t interval){
      ActivePattern = NO_COLOR;
      Index = 0;
      TotalSteps = steps;
      Interval = interval;
    }
    
    // Update the NoColor Pattern
    void NoColorUpdate() {
      ColorSet(Color(0, 0, 0));
      show();
      Increment();
    }

    // Calculate 50% dimmed version of a color (used by ScannerUpdate)
    uint32_t DimColor(uint32_t color){
      // Shift R, G and B components one bit to the right
      uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
      return dimColor;
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color){
      for (int i = 0; i < numPixels(); i++){
        setPixelColor(i, color);
      }
      show();
    }

    // Fade in a certain pixel to a color
    uint32_t FadeInColor(int i, uint32_t color, float percent){
        uint32_t curr_color = getPixelColor(i);
        uint8_t red = Red(curr_color) + Red(color) * percent;
        uint8_t green = Green(curr_color) + Green(color) * percent;
        uint8_t blue = Blue(curr_color) + Blue(color) * percent;
        return Color(red, green, blue);
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color){
      return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color){
      return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color){
      return color & 0xFF;
    }

    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos){
      WheelPos = 255 - WheelPos;
      if (WheelPos < 85){
        return Color(255 - WheelPos * 3, 0, WheelPos * 3);
      }
      else if (WheelPos < 170){
        WheelPos -= 85;
        return Color(0, WheelPos * 3, 255 - WheelPos * 3);
      }
      else{
        WheelPos -= 170;
        return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
      }
    }
};
