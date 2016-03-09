#include <iostream>

#include <delay.h>
#include "logging.h"
#include "robot.h"

stopwatch idp::logging::logging_stopwatch;

int main(int argc, char* argv[]) {
  idp::logging::log_init();
  idp::Robot r;

  IDP_INFO << "Program started." << std::endl;
  
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
    return -1;
  }

  try {
    r.test();
  }
  catch (idp::Robot::LinkError& e) {
    IDP_ERR << "Preliminary test failed. Exiting." << std::endl;
    return -1;
  }

  // Main program execution here.
  try {
    // 1st step: follow line for 0.3m.
    while (r.get_tracking().distance < 0.3) {
      r.line_following();
      r.update_light_sensors();
      r.line_following();
    }
    
    // 2nd step: turn until orientation
  }
  catch (idp::Robot::LineFollowingError& e) {
    // We're lost! Enter recovery mode now.  


    // Don't forget to reset PID control loop before resuming line following.  
  }
  catch (idp::Robot::PositionTrackingError& e) {
    // Discrepancy between position tracking and another subsystem.  
    // That's going to be really tricky to solve, UNLESS we are at a known intersection.  
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
