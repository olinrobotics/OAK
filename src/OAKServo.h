#ifndef OAK_SERVO_H
#define OAK_SERVO_H

#include "ros.h"
#include "std_msgs/Byte.h"
#include <PWMServo.h>
#include "OAK.h"

class OAKServo: private OAK{
public:
  explicit OAKServo(const char* name, const int pin);
  explicit OAKServo(const char* name, const int pin, const int min, const int max);

private:
  ros::Subscriber<std_msgs::Byte, OAKServo> *signalIn;
  std_msgs::Byte servo_signal;
  PWMServo s;
  void servoCB(const std_msgs::Byte &sig);
};

#endif //OAK_SERVO_H
