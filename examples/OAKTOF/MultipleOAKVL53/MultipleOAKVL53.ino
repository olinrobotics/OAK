#include <Arduino.h>
#include <ros.h>
#include <OAKVL53.h>

/******************************************************************************
 *
 * This is an example for using multiple VL53L0X Time of Flight sensors
 * The sensors must be reset and them brought back online one at a time with a
 * a new address because the default address is the same for all of them
 *
 ******************************************************************************/


#define NUM_SENSOR 3

ros::NodeHandle nh;
OAKVL53 *v[NUM_SENSOR];
const String names[] = {"Left", "Center", "Right"};
const byte pins[] = {1,2,3};
const byte addresses[] = {0x31,0x30,0x29};

void setup(){
  nh.initNode(); // Initialize ROS nodehandle
  for(int i = 0; i < NUM_SENSOR; i++){
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
  delay(15); // Give the TOF sensors time to reset
  for(int i = 0; i < NUM_SENSOR; i++){
    digitalWrite(pin[i], HIGH);
    v[i] = new OAKVL53(&nh, names[i], 100, addresses[i]);
  }
}

void loop(){
  nh.spinOnce();
  for(int i = 0; i < NUM_SENSOR; i++){
    v[i]->publish;
  }
}
