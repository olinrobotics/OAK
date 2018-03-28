#include <Arduino.h>
#include <ros.h>
#include <OAKEstop.h>

OAKEstop *e;

void setup() {
  OAK::nh->initNode(); // Initialize ROS nodehandle
  e = new OAKEstop(2, 1);
  e->onStop(stop);
  e->offStop(restart);
  pinMode(13, OUTPUT);
}

void loop() {
  delay(100);
  OAK::nh->spinOnce();
}

void stop(){
  digitalWrite(13, HIGH);
}

void restart(){
  digitalWrite(13, LOW);
}
