#include <ros.h>
#include <std_msgs/String.h>
#include "Emotion.h"
#include <inttypes.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN            8

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      12

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// For RGB_W Neopixel rings, the third parameter needs to be changed as so. Examples of alternative paramenters
// for other versions, see stradtest example provided with the Adafruit library

// Make it NEO_GRBW because the red and green are switched on these neopixels for some reason.
// By setting this value, it can be treated as RGB
//NeoPatterns Ring(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800, NULL);
Emotion e(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);

String curr = "none";
void emotion_cb( const std_msgs::String& rand_num){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  String d = rand_num.data;
  if(curr != d){
    if(d == "praise"){
      curr = "praise";
      e.Praise();
    }else if(d == "gloat"){
      curr = "gloat";
      e.Gloat();
    }else if(d == "heart"){
      curr = "heart";
      e.Blush();
    } else{
      e.NoEmotion();
      curr = "none";
    }
  }
  
}

ros::NodeHandle  nh;
// Make sure the topic you are subscribing to matches to the one that is being published to
ros::Subscriber<std_msgs::String> emotion_sub("/behaviors_cmd", &emotion_cb );
// Run roscore
// Call rosrun rosserial_python serial_node.py /dev/ttyACM0 (or whatever port the Arduino is, check in the bottom right hand corner of the Arduino IDE)
// Call the a random talker function, currently in the catkin_ws/src/beginner_tutorials/scripts/random_talker.py
void setup()
{
  nh.initNode();
  nh.subscribe(emotion_sub);
//  Serial.begin(115200);
//  e.Gloat();
}

void loop()
{
  // Call the emotion update which will update the pattern if needed
  e.Update();
  nh.spinOnce();
  delay(0.5);// set this to 0.5 to sample fast enough for the Python node to not think that the Arduino is not synced (missing data)
//              // http://answers.ros.org/question/11237/rosserial-lost-sync-with-device/
}
