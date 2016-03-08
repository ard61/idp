#include <iostream>

#include "logging.h"
#include "robot.h"
#include <delay.h>

stopwatch idp::logging::logging_stopwatch;

int main(int argc, char* argv[]) {
  idp::logging::log_init();
  idp::Robot r;
  
  r.load_constants();

  if (!r.initialise()) {
    IDP_ERR << "Unable to connect to robot. Exiting." << std::endl;
    return -1;
  }

  try {
    r.configure();
  }
  catch (idp::Robot::LinkError& e) {
    IDP_ERR << "First configuration failed. Exiting." << std::endl;
    //return -1;
  }

  try {
    r.test();
  }
  catch (idp::Robot::LinkError& e) {
    IDP_ERR << "Preliminary test failed. Exiting." << std::endl;
    return -1;
  }
  
  stopwatch test_stopwatch;
  test_stopwatch.start();

  // Main loop here.
  try {  // Normal line-following regime.
    IDP_INFO << "Turning actuator 1 on" << std::endl;
    r.actuator1_on();
    delay(1000);
    
    IDP_INFO << "Turning actuator 2 on" << std::endl;
    r.actuator2_on();
    delay(1000);
    
    IDP_INFO << "Turning actuator 1 off" << std::endl;
    r.actuator1_off();
    delay(1000);
    
    IDP_INFO << "Turning actuator 2 off" << std::endl;
    r.actuator2_off();
  }
  catch (idp::Robot::ActuatorError& e) {
    
    
  }

  catch (idp::Robot::LinkError& e) {
    // Something's gone wrong with the link.  
    if (!r.reinitialise()) {
      IDP_ERR << "Could not recover from link error. Exiting." << std::endl;
      return -1;
    }
    try {
      r.configure();
    }
    catch (idp::Robot::LinkError& e) {
      IDP_ERR << "Could not recover from link error. Exiting." << std::endl;
      return -1;
    }
  }

  return 0;
}
