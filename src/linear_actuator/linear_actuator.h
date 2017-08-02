#ifndef LINEAR_ACTUATOR_H  //include guard
#define LINEAR_ACTUATOR_H

#include "ros.h"
#include ""
#include "config.h"

#ifndef VELOCITY_PIN
#error THE VELOCITY ACTUATOR PIN NEEDS TO BE DEFINED

#ifndef STEERING_PIN
#error THE STEERING ACTUATOR PIN NEEDS TO BE DEFINED

class LinearActuator(){
public:
  static void setup(ros::NodeHandle *nh);
private:
  static ros::Subscriber<> *ackermanSteering;

}
