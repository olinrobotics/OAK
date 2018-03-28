#include <Arduino.h>
#include <ros.h>
#include <OAKServo.h>

OAKServo *s;

void setup(){
  OAK::nh->initNode(); // Initialize ROS nodehandle
  s = new OAKServo("/test_servo", 2);
}

void loop(){
  OAK::nh->spinOnce();
  delay(100);
}
