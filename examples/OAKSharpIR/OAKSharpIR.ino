#include <Arduino.h>
#include <ros.h>
#include <OAKSharpIR.h>

OAKSharpIR *ir;

void setup(){
  OAK::nh->initNode(); // Initialize ROS nodehandle
  ir = new OAKSharpIR("/test_ir", 100, 23);
}

void loop(){
  OAK::nh->spinOnce();
  ir->publish();
}
