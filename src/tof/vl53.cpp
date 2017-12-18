#include "vl53.h"

/******************************************************************************
 * VL53L0X TOF class for OAK (Olin Autonomous Kore)
 * @file vl53.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a publisher an Adafruit VL53L0X TOF Sensor
 *
 * @class VL53 vl53.h "vl53.h"
 ******************************************************************************/

/*
 * Setup function for the class
 *
 * Initializes a publisher with the given name
 * Initializes thee Time of FLight sensor
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the publisher
 * @param[in] del The delay between publishing distances
 * @param[in] address Address of the VL53L0X (default = 0x29)
 */
VL53::VL53(ros::NodeHandle *nh, const char* name, const unsigned int del, const int address = 0x29):del(del){
  dist_pub = new ros::Publisher(name, &dist);
  nh->advertise(*dist_pub);
  tof = Adafruit_VL53L0X(address);
  last_mill = millis();
}

/*
 * Function that publishes at a given rate
 */
void VL53::publish(){
  if(millis()-last_mill >= del){
    tof.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      dist.data = measure.RangeMilliMeter;
      dist_pub.publish(&dist);
    }
  }
}