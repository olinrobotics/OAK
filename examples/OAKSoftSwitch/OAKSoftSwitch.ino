#include <Arduino.h>
#include <ros.h>
#include <OAKSoftSwitch.h>

OAKSoftSwitch *s;

void setup(){
  OAK::nh->initNode(); // Initialize ROS nodehandle
  s = new OAKSoftSwitch("/test_servo", 13);
}

void loop(){
  OAK::nh->spinOnce();
  delay(100);
}
