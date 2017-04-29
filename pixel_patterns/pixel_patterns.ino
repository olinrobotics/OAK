#include <ros.h>
#include <std_msgs/Int16.h>
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


void messageCb( const std_msgs::Int16& rand_num){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  int d = rand_num.data;
//  Serial.println(d);
  if(d == 1){
    e.Praise();
  }else if(d == 2){
    e.Gloat();
  }else{
    e.Blush();
  }
}

ros::NodeHandle  nh;
// Make sure the topic you are subscribing to matches to the one that is being published to
ros::Subscriber<std_msgs::Int16> sub("chatter", &messageCb );
// Run roscore
// Call rosrun rosserial_python serial_node.py /dev/ttyACM0 (or whatever port the Arduino is, check in the bottom right hand corner of the Arduino IDE)
// Call the a random talker function, currently in the catkin_ws/src/beginner_tutorials/scripts/random_talker.py
void setup()
{
  nh.initNode();
  nh.subscribe(sub);
//  Serial.begin(115200);
//  e.Blush();
}

void loop()
{
  // Call the emotion update which will update the pattern if needed
  e.Update();
  nh.spinOnce();
  delay(0.5);// set this to 0.5 to sample fast enough for the Python node to not think that the Arduino is not synced (missing data)
//              // http://answers.ros.org/question/11237/rosserial-lost-sync-with-device/
}
