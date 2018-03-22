/******************************************************************************
 * Sharp IR class for OAK (Olin Autonomous Kore)
 * @file OAKSharpIR.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a publisher a Sharp IR distance sensor
 *
 * @class OAKSharpIR OAKSharpIR.h "OAKSharpIR.h"
 ******************************************************************************/


#include "OAK.h"
#include "OAKSharpIR.h"

/*
 * Constructor for the class
 *
 * Initializes a publisher with the given name
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the publisher
 * @param[in] del The delay between publishing distances
 * @param[in] pin Analog pin that the sensor is attatched to
 */
OAKSharpIR::OAKSharpIR(const char* name, const unsigned int del, const int pin):del(del), pin(pin){
  dist_pub = new ros::Publisher(name, &dist);
  OAK::nh->advertise(*dist_pub);
  last_mill = millis();
}

/*
 * Function that publishes at a given rate
 */
void OAKSharpIR::publish(){
  if(millis()-last_mill >= del){
    dist.data = 64217*pow(analogRead(pin), -1.295);
    dist_pub->publish(&dist);
  }
}