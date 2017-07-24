#include <Arduino.h>
#include "ros.h"
#include "servo.h"

ros::NodeHandle nh;
OAKServo *s;

void setup(){
  nh.initNode(); // Initialize ROS nodehandle
  s = new OAKServo(&nh, "/test_servo", 2);
}

void loop(){
  nh.spinOnce();
  delay(100);
}
