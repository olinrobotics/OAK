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
 * @class OAKEncoder oakencoder.h "oakencoder.h"
 ******************************************************************************/


#include "OAKEncoder.h"

/*
 * Constructor for the class
 *
 * Initializes a publisher with the given name
 * Initializes an encoder with the given pins
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the publisher
 * @param[in] del The delay between publishing distances
 * @param[in] a Pin a of the encoder
 * @param[in] b Pin b of the encoder
 */
OAKEncoder::OAKEncoder(const char* name, const unsigned int del, byte a, byte b){
  encod_pub = new ros::Publisher(name, &count);
  nh->advertise(*encod_pub);
  enc = new Encoder(a, b);
  timer = new Metro(del);
}

/*
 * Function that publishes at a given rate
 */
void OAKEncoder::publish(){
  if(timer->check()){
    count.data = enc->read();
    enc->write(0);
    encod_pub->publish(&count);
  }
}