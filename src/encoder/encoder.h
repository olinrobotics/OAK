#ifndef ROS_ENCODER
#define ROS_ENCODER

#include "ros.h"
#include "std_msgs/Int64.h"
#include "Encoder.h"

class OAKEncoder{
public:
	explicit OAKEncoder(ros::NodeHandle *nh, const char* name, const unsigned int del, byte a, byte b);
private:
	ros::Publisher *encod_pub;
	std_msgs::Int64 count;
	Encoder enc;
	const unsigned int del;
	long last_mill;
};

#endif //ROS_ENCODER
