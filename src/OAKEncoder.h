#ifndef OAK_ENCODER_H
#define OAK_ENCODER_H

#include "ros.h"
#include "std_msgs/Int64.h"
#include <Encoder.h>
#include "OAK.h"

class OAKEncoder: private OAK{
public:
	explicit OAKEncoder(const char* name, const unsigned int del, byte a, byte b);
	void publish();

private:
	ros::Publisher *encod_pub;
	std_msgs::Int64 count;
	Encoder *enc;
};

#endif //OAK_ENCODER_H
