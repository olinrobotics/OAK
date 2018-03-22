#ifndef OAK_SOFT_SWITCH_H
#define OAK_SOFT_SWITCH_H

#include "OAK.h"
#include "ros.h"
#include "std_msgs/Bool.h"

class OAKSoftSwitch: private OAK{
public:
  explicit OAKSoftSwitch(const char* name, const int pin);

private:
  ros::Subscriber<std_msgs::Bool, OAKSoftSwitch> *signalIn;
  const int pin;
  void softCB(const std_msgs::Bool &sig);
};

#endif //OAK_SOFT_SWITCH_H
