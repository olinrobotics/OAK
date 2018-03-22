#ifndef OAK_ESTOP_H
#define OAK_ESTOP_H

#include "ros.h"
#include "std_msgs/Bool.h"

static void dummyFunc() {return;}
extern void attachInterrupt2(uint8_t pin, void (*function)(void*), int mode, void* clas);

class OAKEstop: private OAK{
public:
	explicit OAKEstop(const int pin, const unsigned int debounceTime);
	static void globalStop(void* instance);
	void onStop(void (*func)());
	void offStop(void (*func)());
	bool isStopped();

private:
    ros::Publisher *hardEStop;
	ros::Subscriber<std_msgs::Bool, OAKEstop> *softEStop;
    std_msgs::Bool stopped;
	bool softStopped = false;
	const unsigned int debounceTime;
	const int pin;
	long last_mill;
	void (*stopfunc)() = dummyFunc;
	void (*startfunc)() = dummyFunc;
    void onChange();
	void softStopCB(const std_msgs::Bool &message);
};

#endif //OAK_ESTOP_H