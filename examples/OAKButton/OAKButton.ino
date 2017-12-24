#include <Arduino.h>
#include <ros.h>
#include <OAKButton.h>

ros::NodeHandle nh;
OAKButton *b;

void setup(){
  nh.initNode(); // Initialize ROS nodehandle
  b = new OAKButton(&nh, "/test_button", 0, 100, CHANGE);
  b->onPress(press);
  b->offPress(release);
  pinMode(13, OUTPUT);
}

void loop(){
  nh.spinOnce();
  delay(100);
}

void press(){
  digitalWrite(13, HIGH);
}

void release(){
  digitalWrite(13, LOW);
}
