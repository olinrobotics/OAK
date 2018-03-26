#ifndef OAK_H
#define OAK_H

#include "ros.h"
#include <Time.h>
#include "OAKTimeAlarms.h"

class OAK{
public:
	// const pointer to nodehandle
	static ros::NodeHandle *const nh;
	// const pointer to timealarm
	static OAKTimeAlarmsClass *const alarm;
};

#endif //OAK_H