/******************************************************************************
 * @file        soft_switch.cpp
 * Software  class for OAK (Olin Autonomous Kore)
 * @author      Carl Moser
 * @email       carl.moser@students.olin.edu
 * @version     1.0
 * @date        24/07/17
 ******************************************************************************/


#include "OAKSoftSwitch.h"

/*
 * Constructor for the class
 *
 * Initializes a publisher with the given name
 * attaches the button pin
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the subscriber
 * @param[in] pin Pin software switch
 */
OAKSoftSwitch::OAKSoftSwitch(const char* name, const int pin):pin(pin){
  signalIn = new ros::Subscriber<std_msgs::Bool, OAKSoftSwitch>(name, &OAKSoftSwitch::softCB, this);
  OAK::nh->subscribe(*signalIn);
  pinMode(pin, OUTPUT);
}
/*
 * Callback for the
 */
void OAKSoftSwitch::softCB(const std_msgs::Bool &sig){
  digitalWrite(pin, sig.data);
}
