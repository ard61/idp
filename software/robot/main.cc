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
    // Definition
    const idp::Robot::MotorDemand full_forward(r._constants.cruise_speed, r._constants.cruise_speed);
    
    // 1st step: follow line for 0.3m.
    while (r.get_tracking().distance < 0.3) {
      r.update_tracking();
      r.update_light_sensors();
      r.line_following();
    }
    
    // 2nd step: turn until orientation is -30 degrees to horizontal
    r.turn_until_orientation(-M_PI/6);
    
    // 3rd step: Go forward for 0.1m.
    r.move(full_forward, 0.1);
    
    // 4th step: turn until orientation is 0 again.  
    r.turn_until_orientation(0);
    
    // 5th step: Go forward for 0.2m.
    r.move(full_forward, 0.1);
    
    // 6th step: turn until orientation is 30 degrees.
    r.turn_until_orientation(30);
    
    // 7th step: move forward until line is hit.
    r.move(full_forward);
    while (!r.hit_line()) {
      r.update_tracking();
      r.update_light_sensors();
    }
    
    // 8th step: turn clockwise until aligned with line again.
    r.turn_until_line(false);
    
    // 9th step: follow line until intersection.
    while (!r.at_intersection) {
      r.update_tracking();
      r.update_light_sensors();
      r.line_following();
    }
    
    // 10th step: turn anti-clockwise until line is hit.
    r.turn_until_line(true);
    
    // 11th step: follow line, stopping at the fourth intersection.
    int i = 0;
    while (i<4) {
      r.update_tracking();
      r.update_light_sensors();
      r.line_following();
    }
    
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
