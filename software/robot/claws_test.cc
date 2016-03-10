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
    IDP_INFO << "Closing claws" << std::endl;
    r.claws_close();
    delay(1000);
    IDP_INFO << "Opening claws" << std::endl;
    r.claws_open();
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
