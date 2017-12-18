#include "encoder.h"

/******************************************************************************
 * Encoder class for OAK (Olin Autonomous Kore)
 * @file encoder.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a publisher a rotary encoder
 *
 * @class Encoder encoder.h "encoder.h"
 ******************************************************************************/

/*
 * Setup function for the class
 *
 * Initializes a publisher with the given name
 * Initializes thee Time of FLight sensor
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the publisher
 * @param[in] del The delay between publishing distances
 * @param[in] a Pin a of the encoder
 * @param[in] b Pin b of the encoder
 */
OAKEncoder::OAKEncoder(ros::NodeHandle *nh, const char* name, const unsigned int del, byte a, byte b):del(del){
  encod_pub = new ros::Publisher(name, &count);
  nh->advertise(*encod_pub);
  enc = Encoder(a, b);
  last_mill = millis();
}

/*
 * Function that publishes at a given rate
 */
void OAKEncoder::publish(){
  if(millis()-last_mill >= del){
    count.data = enc.read();
    enc.write(0);
    dist_pub.publish(&dist);
  }
}