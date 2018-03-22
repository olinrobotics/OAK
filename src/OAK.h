#ifndef OAK_H
#define OAK_H

#include "ros.h"
#include <Time.h>
#include <TimeAlarms.h>

class OAK{
public:
	// const pointer to nodehandle
	static ros::NodeHandle *const nh;
	// const pointer to timealarm
	static TimeAlarmsClass *const alarm;
};

#endif //OAK_H