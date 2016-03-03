#include <iostream>

#include "logging.h"
#include "robot.h"
#include "delay.h"

stopwatch idp::logging::logging_stopwatch;

int main(int argc, char* argv[]) {
  idp::logging::log_init();
  idp::Robot r;
  
  IDP_INFO << "Loading constants." << std::endl;
  
  r.load_constants();
  
  IDP_INFO << "Done loading constants." << std::endl;

  if (!r.initialise()) {
    IDP_ERR << "Unable to connect to robot. Exiting." << std::endl;
    return -1;
  }

  try {
    r.configure();
  }
  catch (idp::Robot::LinkError& e) {
    //IDP_ERR << "First configuration failed. Exiting." << std::endl;
    //return -1;
  }

  try {
    r.test();
  }
  catch (idp::Robot::LinkError& e) {
    IDP_ERR << "Preliminary test failed. Exiting." << std::endl;
    return -1;
  }

  idp::Robot::MotorDemand motor_demand(r._constants.max_speed_l, r._constants.max_speed_l);

  try {
    IDP_INFO << "Moving at maximum speed for 10 seconds." << std::endl;
    r.move(motor_demand);
    delay(10000);  // delay 10 seconds
	r.move(idp::Robot::MotorDemand(0,0));
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
