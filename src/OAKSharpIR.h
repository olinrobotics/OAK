#ifdef OAK_SHARP_IR
#define OAK_SHARP_IR

#include "ros.h"
#include "std_msgs/Float32.h"

class OAKSharpIR{
public:
	explicit OAKSharpIR(ros::NodeHandle *nh, const char* name, const unsigned int del, const int pin);
	void publish();

private:
	ros::Publisher *dist_pub;
	std_msgs::Float32 dist;
	const unsigned int del;
	const int pin;
	long last_mill;
};

#endif //OAK_SHARP_IR