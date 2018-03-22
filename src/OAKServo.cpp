/******************************************************************************
 * Servo class for OAK (Olin Autonomous Kore)
 * @file servo.cpp
 * @author Connor Novak
 * @author Carl Moser
 * @email connor@students.olin.edu
 * @version     1.0
 *
 * This is meant to be a modular class for any robot within the lab
 * it controls the position of a servo based on the input from a subscriber
 *
 * @class OAKServo oakservo.h "oakservo.h"
 ******************************************************************************/


#include "OAK.h"
#include "OAKServo.h"

/*
 * Constructor for the class
 *
 * Initializes a subscriber with the given name
 * Initializes a servo
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the subscriber
 * @param[in] pin The pin that the servo is on
 */
OAKServo::OAKServo(const char* name, const int pin):pin(pin){
  signalIn = new ros::Subscriber<std_msgs::Byte, OAKServo>(name, &OAKServo::servoCB, this);
  OAK::nh->subscribe(*signalIn); // tells ROS about the subscriber
  s.attach(pin);
}

/*
 * Constructor for the class
 *
 * Initializes a subscriber with the given name
 * Initializes a servo with the given min and max pulse width
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the subscriber
 * @param[in] pin The pin that the servo is on
 * @param[in] min The min pulse width
 * @param[in] max The max pulse width
 */
OAKServo::OAKServo(const char* name, const int pin, const int min, const int max):pin(pin){
  signalIn = new ros::Subscriber<std_msgs::Byte, OAKServo>(name, &OAKServo::servoCB, this);
  OAK::nh->subscribe(*signalIn); // tells ROS about the subscriber
  s.attach(pin, min, max);
}

/*
 * The callback for the subcriber
 */
void OAKServo::servoCB(const std_msgs::Byte &sig){
  s.write(sig.data);
}
