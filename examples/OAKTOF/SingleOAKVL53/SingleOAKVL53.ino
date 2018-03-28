#include <Arduino.h>
#include <ros.h>
#include <OAKVL53.h>

OAKVL53 *v;

void setup(){
  OAK::nh->initNode(); // Initialize ROS nodehandle
  v = new OAKVL53("/test_tof", 100);
}

void loop(){
  OAK::nh->spinOnce();
  v->publish();
}
