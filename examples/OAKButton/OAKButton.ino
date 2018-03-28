#include <Arduino.h>
#include <ros.h>
#include <OAKButton.h>

OAKButton *b;

void setup(){
  OAK::nh->initNode(); // Initialize ROS nodehandle
  b = new OAKButton("/test_button", 0, 100, CHANGE);
  b->onPress(press);
  b->offPress(release);
  pinMode(13, OUTPUT);
}

void loop(){
  OAK::nh->spinOnce();
  delay(100);
}

void press(){
  digitalWrite(13, HIGH);
}

void release(){
  digitalWrite(13, LOW);
}
