/******************************************************************************
 * OAKEstop class for OAK (Olin Autonomous Kore)
 * @file estop.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.1
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a publisher and subscriber for an estop
 *
 * @class OAKEstop oakestop.h "oakestop.h"
 ******************************************************************************/


#include "OAK.h"
#include "OAKEstop.h"

/*
 * Constructor for the class
 *
 * Initializes a publisher and subsciber
 * attaches the estop pin
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] pin The pin that the servo is on
 * @param[in] debounceTime The debounce time of the estop button
 */
OAKEstop::OAKEstop(const int pin, const unsigned int debounceTime):pin(pin),debounceTime(debounceTime){
  hardEStop = new ros::Publisher("/hardestop", &stopped);
  softEStop = new ros::Subscriber<std_msgs::Bool, OAKEstop>("/softestop", &OAKEstop::softStopCB, this);
  OAK::nh->advertise(*hardEStop);
  OAK::nh->subscribe(*softEStop);
  last_mill = millis();
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt2(digitalPinToInterrupt(pin), &OAKEstop::globalStop, CHANGE, this);
  stopped.data = !digitalRead(pin);
}

/*
 * Global function that calls the object function
 *
 * @param[in] instance Instance of the class
 */
void OAKEstop::globalStop(void *instance){
  static_cast<OAKEstop*>(instance)->onChange();
}

/*
 * Function that checks if the system is estopped
 *
 * @return True if stopped
 */
bool OAKEstop::isStopped(){
  return softStopped|stopped.data;
}

/*
 * Function that runs on a hardware estop
 *
 * If stopping - calls the stopfunc and publishes true
 * If reseting from stop - calls the startfunc and publishes false
 */
void OAKEstop::onChange(){
  if(millis()-last_mill >= debounceTime*50){
    /*if(digitalRead(ESTOP_PIN)){
      stopped.data = true;
      hardEStop->publish(&stopped);
      (*OAKEstop::stopfunc)();
    }
    else{
      stopped.data = false;
      hardEStop->publish(&stopped);
      (*OAKEstop::startfunc)();
    }*/
    stopped.data = !stopped.data;
    hardEStop->publish(&stopped);
    if(stopped.data){
      (*stopfunc)();
    }
    else{
      if(!softStopped)
        (*startfunc)();
    }
    last_mill = millis();
  }
}

/*
 *Function that runs on sofware estop
 */
void OAKEstop::softStopCB(const std_msgs::Bool &message){
  if(message.data){
    (*stopfunc)();
    softStopped = true;
  }
  else{
    softStopped = false;
    (*startfunc)();
    //if(!stopped.data)
      //(*startfunc)();
  }
}

/*
 * Sets the function to run on estop
 *
 * @param[in] func Pointer to the function to run on an estop
 */
void OAKEstop::onStop(void (*func)()){
  stopfunc = func;
}

/*
 * Sets the function to run when un-estoping
 *
 * @param[in] func Pointer to the function to run off an estop
 */
void OAKEstop::offStop(void (*func)()){
  startfunc = func;
}