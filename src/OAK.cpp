#include "OAK.h"

// Init and set nh pointer to nodehandle
ros::NodeHandle n;
ros::NodeHandle *const OAK::nh = &n;
// set alarmclass pointer to existing alarm
OAKTimeAlarmsClass *const OAK::alarm = &Alarm;