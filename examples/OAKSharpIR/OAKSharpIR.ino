#include <Arduino.h>
#include <ros.h>
#include <OAKSharpIR.h>

ros::NodeHandle nh;
OAKSharpIR *ir;

void setup(){
  nh.initNode(); // Initialize ROS nodehandle
  ir = new OAKSharpIR(&nh, "/test_ir", 100, 23);
}

void loop(){
  nh.spinOnce();
  ir->publish();
}
