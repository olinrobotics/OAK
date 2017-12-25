#ifndef OAK_VL53_H
#define OAK_VL53_H

#include <Time.h>
#include "ros.h"
#include "std_msgs/Float32.h"
#include <Adafruit_VL53L0X.h>

class OAKVL53{
public:
	explicit OAKVL53(ros::NodeHandle *nh, const char* name, const unsigned int del, const int address = 0x29);
	void publish();

private:
	ros::Publisher *dist_pub;
	std_msgs::Float32 dist;
	Adafruit_VL53L0X *tof;
	VL53L0X_RangingMeasurementData_t measure;
	const unsigned int del;
	long last_mill;
};

#endif //OAK_VL53_H
