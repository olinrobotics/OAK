#include "servo.h"

/******************************************************************************
 * @file        servo.cpp
 * Servo class for OAK (Olin Autonomous Kore)
 * @author      Connor Novak
 * @author      Carl Moser
 * @email       connor@students.olin.edu
 * @version     1.0
 * @date        24/07/17
 ******************************************************************************/


OAKServo::OAKServo(ros::NodeHandle *nh, const char* name, const int pin):pin(pin){
  signalIn = new ros::Subscriber<std_msgs::Byte, OAKServo>(name, &OAKServo::servoCB, this);
  nh->subscribe(*signalIn); // tells ROS about the subscriber
  s.attach(pin);
}

OAKServo::OAKServo(ros::NodeHandle *nh, const char* name, const int pin, const int min, const int max):pin(pin){
  signalIn = new ros::Subscriber<std_msgs::Byte, OAKServo>(name, &OAKServo::servoCB, this);
  nh->subscribe(*signalIn); // tells ROS about the subscriber
  s.attach(pin, min, max);
}

void OAKServo::servoCB(const std_msgs::Byte &sig){
  s.write(sig.data);
}
