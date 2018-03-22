#ifndef OAK_SHARP_IR_H
#define OAK_SHARP_IR_H

#include "ros.h"
#include "std_msgs/Float32.h"

class OAKSharpIR: private OAK{
public:
	explicit OAKSharpIR(const char* name, const unsigned int del, const int pin);
	void publish();

private:
	ros::Publisher *dist_pub;
	std_msgs::Float32 dist;
	const unsigned int del;
	const int pin;
	long last_mill;
};

#endif //OAK_SHARP_IR_H