#ifndef OAK_SERVO_H
#define OAK_SERVO_H

 #include "ros.h"
 #include "std_msgs/Byte.h"
 #include "config.h"
 #include <PWMServo.h>

class OAKServo{
public:
  explicit OAKServo(ros::NodeHandle *nh, const char* name, const int pin);
  explicit OAKServo(ros::NodeHandle *nh, const char* name, const int pin, const int min, const int max);

private:
  ros::Subscriber<std_msgs::Byte, OAKServo> *signalIn;
  std_msgs::Byte servo_signal;
  const int pin;
  PWMServo s;
  void servoCB(const std_msgs::Byte &sig);
};

#endif //OAK_SERVO_H
