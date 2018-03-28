#ifndef OAK_SHARP_IR_H
#define OAK_SHARP_IR_H

#include "ros.h"
#include "std_msgs/Float32.h"
#include "OAK.h"

class OAKSharpIR: private OAK{
public:
	explicit OAKSharpIR(const char* name, const unsigned int del, const int pin);
	void publish();

private:
	ros::Publisher *dist_pub;
	std_msgs::Float32 dist;
	const int pin;
};

#endif //OAK_SHARP_IR_H