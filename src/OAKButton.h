#ifndef OAK_BUTTON_H
#define OAK_BUTTON_H

#include <Time.h>
#include "ros.h"
#include "std_msgs/Bool.h"

static void dummyFunc() {return;}
extern void attachInterrupt2(uint8_t pin, void (*function)(void*), int mode, void* clas);

class OAKButton{
public:
	explicit OAKButton(ros::NodeHandle *nh, const char* name, const int pin, const unsigned int debounceTime, const int trigger);
	void onPress(void (*func)());
	void offPress(void (*func)());
	static void globalPress(void* instance);

private:
  ros::Publisher *but;
  std_msgs::Bool pressed;
	const unsigned int debounceTime;
	const int pin;
	long last_mill;
  void onChange();
  void (*pressedfunc)() = dummyFunc;
  void (*releasedfunc)() = dummyFunc;
};

#endif //OAK_BUTTON_H
