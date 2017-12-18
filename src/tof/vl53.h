#ifndef VL53_H
#define VL53_H

#include <Time.h>
#include "ros.h"
#include "std_msgs/Float32.h"
#include "Adafruit_VL53L0X.h"

class OAKVL53{
public:
	explicit OAKVL53(ros::NodeHandle *nh, const char* name, const unsigned int del, const int address);

private:
	ros::Publisher *dist_pub;
	std_msgs::Float32 dist;
	Adafruit_VL53L0X tof;
	const unsigned int del;
	long last_mill;
};

#endif //VL53_H
