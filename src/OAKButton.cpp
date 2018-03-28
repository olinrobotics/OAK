/******************************************************************************
 * OAKButton class for OAK (Olin Autonomous Kore)
 * @file button.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a publisher for a button
 *
 * @class OAKButton oakbutton.h "oakbutton.h"
 *
 * @TODO update documentation
 * @TODO look into reading pin state after a delay
 * @TODO optimize for other types of triggers
 ******************************************************************************/

#include "OAKButton.h"

/*
 * Constructor for the class
 *
 * Initializes a publisher with the given name
 * attaches the button pin
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the publisher
 * @param[in] pin Pin of the button
 * @param[in] debounceTime The debounce time for the button
 * @param[in] trigger When the interrupt should be triggered (eg: CHANGE, RISING, FALLING, etc)
 */
OAKButton::OAKButton(const char* name, const int pin, const unsigned int debounceTime, const int trigger):pin(pin){
  but = new ros::Publisher(name, &pressed);
  nh->advertise(*but);
  timer = new Metro(debounceTime);
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt2(digitalPinToInterrupt(pin), &OAKButton::globalPress, trigger, this);
  pressed.data = !digitalRead(pin);
}

/*
 * Global function that calls the object function
 *
 * @param[in] instance Instance of the class
 */
void OAKButton::globalPress(void *instance){
  static_cast<OAKButton*>(instance)->onChange();
}

/*
 * Function that runs on interrupt
 *
 * If pressed - calls the pressedfunc and publishes true
 * If released - calls the releasedfunc and publishes false
 */
void OAKButton::onChange(){
  if(timer->check()){
    pressed.data = !pressed.data;
    but->publish(&pressed);
    if(pressed.data){
      (*pressedfunc)();
    }
    else{
      (*releasedfunc)();
    }
  }
}

/*
 * Sets the function to run on button press
 *
 * @param[in] func Pointer to the function to run when the button is pressed
 */
void OAKButton::onPress(void (*func)()){
  pressedfunc = func;
}

/*
 * Sets the function to run on button release
 *
 * @param[in] func Pointer to the function to run when the button is released
 */
void OAKButton::offPress(void (*func)()){
  releasedfunc = func;
}
