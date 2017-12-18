#include <Arduino.h>
#include "ros.h"
#include "soft_switch.h"

ros::NodeHandle nh;
OAKSoftSwitch *s;

void setup(){
  nh.initNode(); // Initialize ROS nodehandle
  s = new OAKSoftSwitch(&nh, "/test_servo", 13);
}

void loop(){
  nh.spinOnce();
  delay(100);
}
