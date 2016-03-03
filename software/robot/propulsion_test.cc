#include <iostream>

#include "logging.h"
#include "robot.h"
#include "delay.h"

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

  idp::Robot::MotorDemand motor_demand(0.05, 0.05);

  try {
    IDP_INFO << "Moving at speed of 0.05 m/s" << std::endl;
    r.move(motor_demand);
    delay(10000);  // delay 10 seconds
    IDP_INFO << "Turning 90 degrees anticlockwise" << std::endl;
    r.turn(3.1415/4);
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
