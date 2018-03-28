#ifndef OAK_H
#define OAK_H

#include "ros.h"
#include <Time.h>
#include <Metro.h>

class OAK{
public:
	// const pointer to nodehandle
	static ros::NodeHandle *const nh;
	Metro *timer;
};

#endif //OAK_H
