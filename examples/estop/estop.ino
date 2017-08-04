#include <Arduino.h>

//ROS library and messages
#include "ros.h"

//User classes
#include "estop.h"

// ROS variables
ros::NodeHandle nh;
Estop *e;

void setup() {
  nh.initNode(); // Initialize ROS nodehandle
  e = new Estop(&nh, 2, 1);
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
