#include <iostream>

#include <delay.h>
#include <stopwatch.h>

#include "logging.h"
#include "robot.h"

stopwatch idp::logging::logging_stopwatch;


void move_from_start_to_first_pickup_site(idp::Robot& r) {
  const idp::Robot::MotorDemand full_forward(r._constants.cruise_speed, r._constants.cruise_speed);
  const idp::Robot::MotorDemand full_reverse(-r._constants.cruise_speed, -r._constants.cruise_speed);

  
  r.print_tracking();

  IDP_INFO << "Following line until intersection" << std::endl;
  r.line_following_until_intersection();

  IDP_INFO << "Following line for 55 cm" << std::endl;
  r.line_following(0.55);
  
  IDP_INFO << "Turning clockwise by 30 degrees" << std::endl;
  r.turn(-M_PI/6);
  r.print_tracking();
  
  IDP_INFO << "Moving forward for 25 cm" << std::endl;
  r.move(full_forward, 0.25);
  r.print_tracking();
  
  IDP_INFO << "Turning anticlockwise by 30 degrees" << std::endl;
  r.turn(M_PI/6);
  r.print_tracking();
  
  IDP_INFO << "Moving forward until line is hit" << std::endl;
  r.move_until_hit_line(full_forward);
  r.print_tracking();
  
  IDP_INFO << "Turning clockwise until line is hit" << std::endl;
  r.turn_until_line(false);
  r.print_tracking();
  
  IDP_INFO << "Following line for 40 cm" << std::endl;
  r.line_following(0.45);
  r.print_tracking();
  
  IDP_INFO << "Moving forward for 15 cm" << std::endl;
  r.move(full_forward, 0.2);
  r.print_tracking();
  
  IDP_INFO << "Following line until intersection" << std::endl;
  r.line_following_until_intersection();
  r.print_tracking();
  
  IDP_INFO << "Following line for 15 cm" << std::endl;
  r.line_following(0.2);
  
  IDP_INFO << "Moving forward for 15 cm" << std::endl;
  r.move(full_forward, 0.2);
  r.print_tracking();
  
  IDP_INFO << "Following line until intersection" << std::endl;
  r.line_following_until_intersection();
  r.print_tracking();
  
  IDP_INFO << "Turning anticlockwise for 90 degrees" << std::endl;
  r.turn_until_line(true);
  r.print_tracking();
  
  //IDP_INFO << "Following line backwards until intersection" << std::endl;
  //r.line_following_backwards_until_intersection();
}

void move_from_zero_to_first_pickup_site(idp::Robot &r) {
  IDP_INFO << "Following line unti intersection" << std::endl;
  r.line_following_until_intersection();
  r.print_tracking();
  
  IDP_INFO << "Turning anticlockwise until line is hit" << std::endl;
  r.turn_until_line(true);
  r.print_tracking();
}

void move_from_last_pickup_site_to_frying_pan(idp::Robot &r) {
  const idp::Robot::MotorDemand full_forward(r._constants.cruise_speed, r._constants.cruise_speed);

  IDP_INFO << "Following line for 35 cm" << std::endl;
  r.line_following(0.35);
  r.print_tracking();
  
  IDP_INFO << "Turning for 30 degrees anticlockwise" << std::endl;
  r.turn(M_PI/6);
  r.print_tracking();
  
  IDP_INFO << "Moving until line is hit" << std::endl;
  r.move_until_hit_line(full_forward);

  // Might need to do some alignment here
}

void move_from_last_pickup_site_to_chick_dropoff_site(idp::Robot &r) {
  const idp::Robot::MotorDemand full_forward(r._constants.cruise_speed, r._constants.cruise_speed);

  IDP_INFO << "Following line for 35 cm" << std::endl;
  r.line_following(0.35);
  r.print_tracking();
  
  IDP_INFO << "Turning for 30 degrees anticlockwise" << std::endl;
  r.turn(M_PI/6);
  r.print_tracking();
  
  IDP_INFO << "Moving until line is hit" << std::endl;
  r.move_until_hit_line(full_forward);
  
  IDP_INFO << "Moving until line is hit" << std::endl;
  r.move_until_hit_line(full_forward);
  
  
}

void move_from_last_pickup_site_to_egg_box(idp::Robot &r) {

}

void move_from_frying_pan_to_egg_box(idp::Robot &r) {

}

void move_from_chick_dropoff_site_to_egg_box(idp::Robot &r) {

}

void move_from_egg_box_to_start(idp::Robot &r) {

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
    for (int eggs_picked = 0; eggs_picked < 5; eggs_picked++) {
      move_from_start_to_first_pickup_site(r);
      
      /*
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
        move_from_last_pickup_site_to_egg_box(r);
        r.claws_open();
        r.eject();
      }

      else {
        r.crack();
        r.claws_open();
        delay(500);
        r.claws_close();

        if (r.content_is_white()) {
          move_from_last_pickup_site_to_frying_pan(r);
          r.eject();
          move_from_frying_pan_to_egg_box(r);
          r.release_claws();
          r.eject();
        }

        else {
          move_from_last_pickup_site_to_chick_dropoff_site(r);
          r.eject();
          move_from_chick_dropoff_site_to_egg_box(r);
          r.release_claws();
          r.eject();
        }
      }
      move_from_egg_box_to_start(r);

      if (eggs_picked == 4 and sw.read() > 240000) {
        // We have picked up four eggs, but there is less than 1 minute left, so stay at base.
        break;
      }
      */
      break;
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
