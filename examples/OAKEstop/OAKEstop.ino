#include <Arduino.h>
#include <ros.h>
#include <OAKEstop.h>

// ROS variables
ros::NodeHandle nh;
OAKEstop *e;

void setup() {
  nh.initNode(); // Initialize ROS nodehandle
  e = new OAKEstop(&nh, 2, 1);
  e->onStop(stop);
  e->offStop(restart);
  pinMode(13, OUTPUT);
}

void loop() {
  delay(100);
  nh.spinOnce();
}

void stop(){
  digitalWrite(13, HIGH);
}

void restart(){
  digitalWrite(13, LOW);
}
