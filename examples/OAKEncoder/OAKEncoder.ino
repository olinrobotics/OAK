#include <Arduino.h>
#include <ros.h>
#include <OAKEncoder.h>

ros::NodeHandle nh;
OAKEncoder *e;

void setup(){
  nh.initNode(); // Initialize ROS nodehandle
  e = new OAKEncoder(&nh, "/test_encoder", 100, 2, 3);
}

void loop(){
  nh.spinOnce();
  e->publish();
}
