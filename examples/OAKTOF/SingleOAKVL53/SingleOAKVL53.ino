#include <Arduino.h>
#include <ros.h>
#include <OAKVL53.h>

ros::NodeHandle nh;
OAKVL53 *v;

void setup(){
  nh.initNode(); // Initialize ROS nodehandle
  v = new OAKVL53(&nh, "/test_tof", 100);
}

void loop(){
  nh.spinOnce();
  v->publish();
}
