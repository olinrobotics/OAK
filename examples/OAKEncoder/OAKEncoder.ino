#include <Arduino.h>
#include <ros.h>
#include <OAKEncoder.h>

OAKEncoder *e;

void setup(){
  OAK::nh->initNode(); // Initialize ROS nodehandle
  e = new OAKEncoder("/test_encoder", 100, 2, 3);
}

void loop(){
  OAK::nh->spinOnce();
  e->publish();
}
