#include <iostream>

#include <delay.h>
#include <stopwatch.h>

#include "logging.h"
#include "robot.h"

stopwatch idp::logging::logging_stopwatch;

void move_from_start_to_first_pickup_site() {
  const idp::Robot::MotorDemand::full_forward(r._constants.cruise_speed, r._constants.cruise_speed);

  // 1st step: follow line for 0.3m.
  r.line_following(0.3);
  
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
  r.move_until_hit_line(full_forward);
  
  // 8th step: turn clockwise until aligned with line again.
  r.turn_until_line(false);
  
  // 9th step: follow line until intersection.
  r.line_following_until_intersection();
  
  // 10th step: turn anti-clockwise until line is hit.
  r.turn_until_line(true);
}

void move_from_last_pickup_site_to_frying_pan() {
}

void move_from_last_pickup_site_to_chick_dropoff_site() {

}

void move_from_last_pickup_site_to_egg_box() {

}

void move_from_frying_pan_to_egg_box() {

}

void move_from_chick_dropoff_site_to_egg_box() {

}

void move_from_egg_box_to_start() {

}

int main(int argc, char* argv[]) {
  idp::logging::log_init();
  idp::Robot r;

  stopwatch sw;

  sw.start();

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
    for (int eggs_picked = 0; i < 5; i++)
      move_from_start_to_first_pickup_site();
      
      // Follow line, stopping at the 4-eggs_picked intersection.
      for (int i = 0; i < 4 - eggs_picked; i++) {
        r.line_following_until_intersection();
      }

      // Pick egg up.
      r.claws_close();

      // Follow line for the rest of intersections. 
      for (int i = 0; i < eggs_picked; i++) {
        r.line_following_until_intersection();
      }

      if (r.egg_is_fake()) {
        move_from_last_pickup_site_to_egg_box();
        r.claws_open();
        r.eject();
      }

      else {
        r.crack();
        r.claws_open();
        delay(500);
        r.claws_close();

        if (r.content_is_white()) {
          move_from_last_pickup_site_to_frying_pan();
          r.eject();
          move_from_frying_pan_to_egg_box();
          r.release_claws();
          r.eject();
        }

        else {
          move_from_last_pickup_site_to_chick_dropoff_site();
          r.eject();
          move_from_chick_dropoff_site_to_egg_box();
          r.release_claws();
          r.eject();
        }
      }
      move_from_egg_box_to_start();

      if (eggs_picked == 4 and sw.read() > 240000) {
        // We have picked up four eggs, but there is less than 1 minute left, so stay at base.
        return 0;
      }
    }
    return 0;
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
