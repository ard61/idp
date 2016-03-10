#include <fstream>
#include <iostream>

#include <delay.h>

#include "logging.h"
#include "robot.h"


idp::Robot::Robot() : actuator1_is_on(false), actuator2_is_on(false) {
  _sw.start();

  _light_sensors_history = new LightSensorsHistory();
  _tracking_history = new TrackingHistory();
  _map = new idp::Map();
  
  at_intersection = false;
  newly_arrived_at_intersection = false;
  claws_are_open = true; // We always assume that robot starts with claws open.
}

idp::Robot::~Robot() {
  _sw.stop();
  delete _light_sensors_history;
  delete _tracking_history;
  delete _map;
}

void idp::Robot::load_constants() {
  // Initialisation
  _constants.robot_num = 9;  // Wireless interface identifier for our robot
  _constants.map_file = "playing_area.map";  // playing area map filename
  _constants.num_tests = 100;  // Number of tests in each test command

  // Propulsion
  _constants.half_axle_length = 0.15;  // half-axle length
  _constants.max_speed_l = 0.135;  // maximum speed of left robot motor
  _constants.max_speed_r = 0.135;  // maximum speed of right robot motor
  _constants.curve_curvature = 1.67;  // Curvature of the curved white line paths
  _constants.cruise_speed = 0.12;  // Standard speed of the robot
  _constants.ramp_time = 255;  // Ramp time for robot motors
  _constants.turn_until_line_threshold_angle = 0.25;  // Threshold angle after which the robot will start detecting for a new intersection.
  _constants.turn_until_line_max_angle = M_PI/2;  // Default max. angle the robot will turn through if it does not hit a white line.  Prevents infinite 360 deg. turns.
  
  // Tracking
  _constants.initial_position_x = -0.9;  // Robot initial position, x-coordinate
  _constants.initial_position_y = 0;  // Robot initial position, y-coordinate
  _constants.initial_orientation = 0;  // Robot initial orientation
  _constants.go_blind_tolerance = 0.02;
  _constants.turn_until_orientation_tolerance = 0.05; // radians

  // Line following
  _constants.line_following_kp = 1.2;  // Proportional control loop coefficient, gives curvature as a function of error. 
  _constants.intersection_threshold_distance = 0.05; // We are 'close' to an intersection if distance smaller than this value.
  
  // Egg processing
  _constants.claw_open_time = 1;

  // Put the first value of position and speed in the _tracking_history data structure
  idp::Timestamp<idp::Robot::Tracking> tracking;
  tracking.ms = _sw.read();
  tracking.value.speed = 0;
  tracking.value.curvature = 0;
  tracking.value.position = idp::Vector2d(_constants.initial_position_x, _constants.initial_position_y);
  tracking.value.orientation = _constants.initial_orientation;
  tracking.value.distance = 0;

  _tracking_history->push_back(tracking);
  
  _map->populate(_constants.map_file.c_str());
}

bool idp::Robot::initialise() {
  #ifndef __arm__
  int rc = _rlink.initialise(_constants.robot_num);
  #else
  int rc = _rlink.initialise();
  #endif

  if (!rc) { // setup the link
    IDP_ERR << "Cannot initialise link." << std::endl;
    _rlink.print_errs("    ");
    return false;
  }

  IDP_INFO << "Link initialisation complete." << std::endl;

  return true;
}

bool idp::Robot::reinitialise() {
  if (!_rlink.reinitialise()) { // 
    IDP_ERR << "Could not reinitialise following link error." << std::endl;
    _rlink.print_errs("    ");
    return false;
  }

  IDP_INFO << "Link reinitialisation complete." << std::endl;

  return true;
}

void idp::Robot::configure() {
  // Set motor ramp time
  if (!_rlink.command(RAMP_TIME, _constants.ramp_time)) {
    IDP_ERR << "Could not set motor ramp time on microcontroller." << std::endl;
    throw idp::Robot::LinkError();
  }

  // Set bits 0-3 and 6-7 on light sensor board (address 101)
  if (!_rlink.command(WRITE_PORT_5, 0xCF)) {
    IDP_ERR << "Could not set up I2C bus to read from light sensor board." << std::endl;
    throw idp::Robot::LightSensorError();
  }
  
  actuator1_off();
  actuator2_off();
  led1_off();
  led2_off();
  
  // Take initial sensor readings from light sensors to start populating history. 
  update_light_sensors();
  update_light_sensors();
}

bool idp::Robot::test() {
  stopwatch sw;
  int val;
  
  sw.start();
  for (int i = 0; i < _constants.num_tests; i++) {
    val = _rlink.request(TEST_INSTRUCTION);   // send test instruction
  }
  int etime = sw.stop();

  if (val == TEST_INSTRUCTION_RESULT) {     // check result
    IDP_INFO << "Test passed, " << "each test took on average " 
              << (double) etime/_constants.num_tests << " milliseconds." << std::endl;
    return true;
  }
  
  else {
    std::cout << _sw.read () << ": " << std::setw(10) << "ERROR: " << "An error occured during preliminary test." << std::endl;
    throw idp::Robot::LinkError();
  }
  
  return false;
}

void idp::Robot::move(idp::Robot::MotorDemand motor_demand) {
  // Preliminary bounds check
  if (motor_demand.speed_l > _constants.max_speed_l) {
	  motor_demand.speed_r *= _constants.max_speed_l / motor_demand.speed_l;
	  motor_demand.speed_l = _constants.max_speed_l;
	  IDP_WARN << "Motor demand egg-ceded max speed, scaled down." << std::endl;
  }
  if (motor_demand.speed_r > _constants.max_speed_r) {
	  motor_demand.speed_l *= _constants.max_speed_r / motor_demand.speed_r;
	  motor_demand.speed_r = _constants.max_speed_r;
	  IDP_WARN << "Motor demand egg-ceded max speed, scaled down." << std::endl;
  }
  if (motor_demand.speed_l < -_constants.max_speed_l) {
	  motor_demand.speed_r *= _constants.max_speed_l / motor_demand.speed_l;
      motor_demand.speed_l = -_constants.max_speed_l;
	  IDP_WARN << "Motor demand egg-ceded max speed, scaled down." << std::endl;
  }
  if (motor_demand.speed_r < -_constants.max_speed_r) {
	  motor_demand.speed_l *= _constants.max_speed_r / motor_demand.speed_r;
	  motor_demand.speed_r = -_constants.max_speed_r;
	  IDP_WARN << "Motor demand egg-ceded max speed, scaled down." << std::endl;
  }

  // TODO: Put negative signs here if needed when final motor orientation becomes known
  int motor1_demand = -127 * motor_demand.speed_l / _constants.max_speed_l;  // Between -127 and 127.
  int motor2_demand = 127 * motor_demand.speed_r / _constants.max_speed_r;

  unsigned char speed_l, speed_r;
  if (motor1_demand >= 0) speed_l = motor1_demand;
  else speed_l = 128 - motor1_demand;

  if (motor2_demand >= 0) speed_r = motor2_demand;
  else speed_r = 128 - motor2_demand;

  /*
  Motor wiring:

  MOTOR_1 -> red
  MOTOR_2 -> green -> right motor
  MOTOR_3 -> blue -> left motor
  MOTOR_4 -> yellow
  */
  
  IDP_INFO << "Sending command to motor 3: " << (int)speed_l << std::endl;
  IDP_INFO << "Sending command to motor 2: " << (int)speed_r << std::endl;
  
  if (!_rlink.command(MOTOR_3_GO, speed_l)
      or !_rlink.command(MOTOR_2_GO, speed_r)) {
    IDP_ERR << "Error occurred in sending command to motor. " << std::endl;
    throw idp::Robot::LinkError();
  }
}

void idp::Robot::move(const idp::Robot::MotorDemand motor_demand, const double distance) {
  int delta_t_ms = 2 * distance / (motor_demand.speed_l + motor_demand.speed_r) * 1000;
  
  idp::Robot::MotorDemand zero(0,0);

  int current_ms = _sw.read();
  move(motor_demand);

  while (_sw.read() - current_ms < delta_t_ms - 10) { // 10 ms to account for lag
    update_tracking();
    update_light_sensors();
    delay(10);
  }

  move(zero);
}

void idp::Robot::turn(const double angle, const double angular_velocity) {
  int delta_t_ms;
  double speed;
  if (angle*angular_velocity >= 0) {
    delta_t_ms = angle / angular_velocity * 1000;
    speed = angular_velocity * _constants.half_axle_length;
  }
  else {
    delta_t_ms = -angle / angular_velocity * 1000;
    speed = -angular_velocity * _constants.half_axle_length;
  }

  IDP_INFO << "Turning " << angle << " radians means setting each motor at a speed of "
           << speed << " m/s for " << (double)(delta_t_ms)/1000.0 << " seconds." << std::endl;

  // If angular velocity is positive (anticlockwise), then left motor should go backwards.  
  idp::Robot::MotorDemand motor_demand(-speed, speed);
  idp::Robot::MotorDemand zero(0,0);

  int current_ms = _sw.read();
  move(motor_demand);

  while (_sw.read() - current_ms < delta_t_ms) {
    update_tracking();
    update_light_sensors();
  }

  move(zero);
}

void idp::Robot::turn(const double angle) {
  turn(angle, _constants.cruise_speed / _constants.half_axle_length);
}

void idp::Robot::turn_until_line(const bool anticlockwise,
                                 const double threshold_angle, 
                                 const double max_angle, 
                                 const double angular_velocity) {
  int delta_t_ms;
  int threshold_ms;
  
  double speed;
  if (anticlockwise) {
    delta_t_ms = max_angle / angular_velocity * 1000;
    threshold_ms = threshold_angle / angular_velocity * 1000;
    speed = angular_velocity * _constants.half_axle_length;
  }
  else {
    delta_t_ms = -max_angle / angular_velocity * 1000;
    threshold_ms = threshold_angle / angular_velocity * 1000;
    speed = -angular_velocity * _constants.half_axle_length;
  }

  // If angular velocity is positive (anticlockwise), then left motor should go backwards.  
  idp::Robot::MotorDemand motor_demand(-speed, speed);
  idp::Robot::MotorDemand zero(0,0);

  int current_ms = _sw.read();
  move(motor_demand);

  double turned_radians;
  
  int elapsed_ms;

  while ((elapsed_ms = _sw.read() - current_ms) < delta_t_ms) {
    update_tracking();
    update_light_sensors();
    if (_light_sensors_history->back().value.values.rear == 1 and elapsed_ms > threshold_ms) {
      IDP_INFO << "Hit line while turning." << std::endl;
      if (anticlockwise) {
        turned_radians = (elapsed_ms) * angular_velocity;
        break;
      }
    }
  }

  move(zero);
  IDP_INFO << "Turned for " << turned_radians << " radians." << std::endl;
}

void idp::Robot::turn_until_line(bool anticlockwise) {
  turn_until_line(anticlockwise, 
                  _constants.turn_until_line_threshold_angle, 
                  _constants.turn_until_line_max_angle, 
                  _constants.cruise_speed / _constants.half_axle_length);
}

void idp::Robot::turn_until_orientation(const double final_orientation, const double angular_velocity) {
  double speed;
  if ((final_orientation - get_tracking().orientation > 0) 
     or (final_orientation - get_tracking().orientation < -M_PI)) {
    speed = angular_velocity * _constants.half_axle_length;
  }
  else {
    speed = -angular_velocity * _constants.half_axle_length;
  }

  // If angular velocity is positive (anticlockwise), then left motor should go backwards.  
  idp::Robot::MotorDemand motor_demand(-speed, speed);
  idp::Robot::MotorDemand zero(0,0);

  move(motor_demand);

  while ((get_tracking().orientation > final_orientation + _constants.turn_until_orientation_tolerance)
        or (get_tracking().orientation < final_orientation - _constants.turn_until_orientation_tolerance)) {
    update_tracking();
    update_light_sensors();
  }
  move(zero);
}

void idp::Robot::turn_until_orientation(const double final_orientation) {
  turn_until_orientation(final_orientation, _constants.cruise_speed/_constants.half_axle_length);
}

void idp::Robot::update_tracking() {
  idp::Timestamp<idp::Robot::Tracking> tracking;

  // Get velocity information from motors
  int speed_1 = _rlink.request(MOTOR_1);
  int speed_2 = _rlink.request(MOTOR_2);

  if (speed_1 == REQUEST_ERROR or speed_2 == REQUEST_ERROR) {
    IDP_ERR << "Error occurred in requesting speed information from motors. " << std::endl;
    throw idp::Robot::LinkError();
  }

  tracking.ms = _sw.read();

  if (speed_1 > 127) speed_1 = 128 - speed_1;
  if (speed_2 > 127) speed_2 = 128 - speed_2;

  // TODO: Adjust this when final robot orientation is known
  speed_1 = -speed_1;

  double speed_l = idp::Robot::_constants.max_speed_l / 128 * speed_1;
  double speed_r = idp::Robot::_constants.max_speed_r / 128 * speed_2;

  // Calculate new speed, curvature, position and orientation using second-order Euler integrator
  tracking.value.speed = (speed_l + speed_r) / 2;
  tracking.value.curvature = (speed_r - speed_l) / (2 * _constants.half_axle_length * tracking.value.speed);

  double delta_t = tracking.ms - _tracking_history->back().ms;

  double prev_orientation = _tracking_history->back().value.orientation;
  //double prev_curvature = _tracking_history->back().value.curvature;
  double prev_speed = _tracking_history->back().value.speed;
  idp::Vector2d prev_position = _tracking_history->back().value.position;
  double prev_distance = _tracking_history->back().value.distance;

  // Don't integrate orientation from curvature as the latter is potentially infinite.  
  tracking.value.orientation = prev_orientation + delta_t * (speed_r - speed_l) / (2 * _constants.half_axle_length);
  while (tracking.value.orientation < -M_PI) tracking.value.orientation += 2*M_PI;
  while (tracking.value.orientation >= M_PI) tracking.value.orientation -= 2*M_PI;
  tracking.value.position = prev_position + delta_t * ((tracking.value.speed + prev_speed) / 2) * idp::Vector2d::from_polar(1, (tracking.value.orientation + prev_orientation) / 2);
  tracking.value.distance = prev_distance + delta_t * (tracking.value.speed + prev_speed) / 2;
  // Store into the _tracking data structure

  _tracking_history->push_back(tracking);
}

void idp::Robot::print_tracking() {
  IDP_INFO << "Position: x=" << _tracking_history->back().value.position.x
           << ", y=" << _tracking_history->back().value.position.y << std::endl;
  IDP_INFO << "Orientation: " << _tracking_history->back().value.orientation << "rads, anticlockwise from horizontal" << std::endl;
  IDP_INFO << "Speed: " << _tracking_history->back().value.speed << std::endl;
  IDP_INFO << "Path curvature: " << _tracking_history->back().value.speed << std::endl;
  IDP_INFO << "Distance travelled: " << _tracking_history->back().value.distance << std::endl;
}

idp::Robot::Tracking idp::Robot::get_tracking() {
  return _tracking_history->back().value;
}

void idp::Robot::update_light_sensors() {
  idp::Timestamp<idp::Robot::LightSensors> light_sensors;
  int rc = _rlink.request(READ_PORT_5);

  if (rc == REQUEST_ERROR) {
    IDP_ERR << "Error occurred when reading from light sensor board (I2C address 5). " << std::endl;
    throw idp::Robot::LightSensorError();
  }

  light_sensors.ms = _sw.read();
  light_sensors.value.cat = rc;

  _light_sensors_history->push_back(light_sensors);
}

void idp::Robot::line_following() {
  idp::Robot::LightSensorValues line_sensors = _light_sensors_history->back().value.values;

  bool close_to_intersection = (_map->distance_from_intersection(_tracking_history->back().value.position) > _constants.intersection_threshold_distance);

  if (line_sensors.front_left == 0 
      and line_sensors.front_centre == 0 
      and line_sensors.front_right == 0) {
    IDP_INFO << "All black" << std::endl;
    move(MotorDemand(0,0));
    throw idp::Robot::LineFollowingError();
  }
  
  else if (line_sensors.front_left == 0 
      and line_sensors.front_centre == 1 
      and line_sensors.front_right == 0) {
        
    IDP_INFO << "All black except middle sensor" << std::endl;

    idp::Robot::LightSensorValues prev_line_sensors = _light_sensors_history->at(_light_sensors_history->size() - 1).value.values;
    
    if (prev_line_sensors.front_left == 1 
        and prev_line_sensors.front_right == 0) {
      turn_until_line(false);
    }
      
    else if (prev_line_sensors.front_right == 1
             and prev_line_sensors.front_left == 0) {
      turn_until_line(true);
    }
    
    else move(MotorDemand(_constants.cruise_speed, _constants.cruise_speed));
  }

  else if (line_sensors.front_left == 1
      and line_sensors.front_centre == 1
      and line_sensors.front_right == 0) {
    IDP_INFO << "Left and center sensors are white" << std::endl;
    move(calculate_demand(1));
  }

  else if (line_sensors.front_left == 1
      and line_sensors.front_centre == 0 
      and line_sensors.front_right == 0) {
    IDP_INFO << "Far left sensor is white" << std::endl;
    move(calculate_demand(2));
  }

  else if (line_sensors.front_left == 0
      and line_sensors.front_centre == 1 
      and line_sensors.front_right == 1) {
    IDP_INFO << "Right and center sensors are white" << std::endl;
    move(calculate_demand(-1));
  }

  else if (line_sensors.front_left == 0
      and line_sensors.front_centre == 0 
      and line_sensors.front_right == 1) {
    IDP_INFO << "Far right sensor is white" << std::endl;
    move(calculate_demand(-2));
  }
  
  else if (line_sensors.front_left == 1
      and line_sensors.front_centre == 0
      and line_sensors.front_right == 1) {
    if (!close_to_intersection) {
      IDP_ERR << "Position tracking says we are far from intersection but we read front left and front right sensors white from light sensor board." << std::endl;
      throw idp::Robot::PositionTrackingError();
    }
    else {
      IDP_WARN << "Robot is at an odd angle to line, starting recovery procedure." << std::endl;
      throw idp::Robot::LineFollowingError();
    }  
  }

  else if (line_sensors.front_left == 1
      and line_sensors.front_centre == 1
      and line_sensors.front_right == 1) {
    if (!close_to_intersection) {
        IDP_WARN << "Robot is at an odd angle to line, starting recovery procedure." << std::endl;
        throw idp::Robot::LineFollowingError();
    }
    else {
      // We hit that intersection!!!
      newly_arrived_at_intersection = true;
      if (at_intersection == true) newly_arrived_at_intersection = false;
      at_intersection = true;
      IDP_INFO << "We have reached the intersection. " << std::endl;
      move(MotorDemand(_constants.cruise_speed, _constants.cruise_speed));
    }
  }
}

void idp::Robot::line_following(double distance) {
  double prev_distance = get_tracking().distance;
  while (get_tracking().distance - prev_distance < distance) {
    update_tracking();
    update_light_sensors();
    line_following();
  }
}

void idp::Robot::line_following_until_intersection() {
  line_following(0.05); // Start following line for 5 cm to get clear of previous intersection
  while (!newly_arrived_at_intersection) {
    update_tracking();
    update_light_sensors();
    line_following();
  }
}

void idp::Robot::tracking_analysis() {
  // To be performed after line following tells us whether we have arrived at an intersection or not. 
  if (at_intersection)
  {
  	if (current_line != -1) {
  		// We have just reached a new intersection.  Update the map state. 
  	}
  }
}

idp::Robot::MotorDemand idp::Robot::calculate_demand(double error) {
  idp::Robot::MotorDemand motor_demand;
  
  double target_curvature = error * _constants.line_following_kp;

  motor_demand.speed_l = _constants.cruise_speed * (1 - _constants.half_axle_length * (target_curvature));
  motor_demand.speed_r = _constants.cruise_speed * (1 + _constants.half_axle_length * (target_curvature));

  return motor_demand;
}

bool idp::Robot::hit_line() {
  if ((_light_sensors_history->back().value.values.front_centre) == 1) return true;
  else return false;
}

void idp::Robot::move_until_hit_line(const idp::Robot::MotorDemand motor_demand) {
  move(motor_demand);
  while (!hit_line()) {
    update_tracking();
    update_light_sensors();
  }
  move(MotorDemand(0,0));
}

void idp::Robot::go_blind(Vector2d target_position) {
  double direction = (target_position - _tracking_history->back().value.position).angle();
  double distance = (target_position - _tracking_history->back().value.position).abs();

  double angle = direction - _tracking_history->back().value.orientation;
  while (angle > M_PI) angle -= 2*M_PI;
  while (angle <= -M_PI) angle += 2*M_PI;
  turn(angle);
  move(idp::Robot::MotorDemand(_constants.cruise_speed, _constants.cruise_speed), distance);
}

void idp::Robot::go_blind_iter(Vector2d target_position) {
  // Iterates until we come close enough to the target position.
  while ((target_position - _tracking_history->back().value.position).abs() > _constants.go_blind_tolerance) {
    go_blind(target_position);
  }
}

void idp::Robot::recovery() {
  // First we need to know which line we are on
}

void idp::Robot::led1_on() {
  if (led2_is_on) {
    if (!_rlink.command(WRITE_PORT_5, 0xCF bitor 0x00)) {
      IDP_ERR << "Could not send command to LED board" << std::endl;
      throw idp::Robot::LightSensorError();
    }
  }
  else {
    if (!_rlink.command(WRITE_PORT_5, 0xCF bitor 0x20)) {
      IDP_ERR << "Could not send command to LED board" << std::endl;
      throw idp::Robot::LightSensorError();
    }
  }
  led1_is_on = true;
}

void idp::Robot::led1_off() {
  if (led2_is_on) {
    if (!_rlink.command(WRITE_PORT_5, 0xCF bitor 0x10)) {
      IDP_ERR << "Could not send command to LED board" << std::endl;
      throw idp::Robot::LightSensorError();
    }
  }
  else {
    if (!_rlink.command(WRITE_PORT_5, 0xCF bitor 0x30)) {
      IDP_ERR << "Could not send command to LED board" << std::endl;
      throw idp::Robot::LightSensorError();
    }
  }
  led1_is_on = false;
}

void idp::Robot::led2_on() {
  if (led1_is_on) {
    if (!_rlink.command(WRITE_PORT_5, 0xCF bitor 0x00)) {
      IDP_ERR << "Could not send command to LED board" << std::endl;
      throw idp::Robot::LightSensorError();
    }
  }
  else {
    if (!_rlink.command(WRITE_PORT_5, 0xCF bitor 0x10)) {
      IDP_ERR << "Could not send command to LED board" << std::endl;
      throw idp::Robot::LightSensorError();
    }
  }
  led2_is_on = true;
}

void idp::Robot::led2_off() {
  if (led1_is_on) {
    if (!_rlink.command(WRITE_PORT_5, 0xCF bitor 0x20)) {
      IDP_ERR << "Could not send command to LED board" << std::endl;
      throw idp::Robot::LightSensorError();
    }
  }
  else {
    if (!_rlink.command(WRITE_PORT_5, 0xCF bitor 0x30)) {
      IDP_ERR << "Could not send command to LED board" << std::endl;
      throw idp::Robot::LightSensorError();
    }
  }
  led2_is_on = false;
}


void idp::Robot::actuator1_on() {
  if (actuator2_is_on) {
    if (!_rlink.command(WRITE_PORT_2, 0x00)) {
      IDP_ERR << "Could not send command to actuator board" << std::endl;
      throw idp::Robot::ActuatorError();
    }
  }
  else {
    if (!_rlink.command(WRITE_PORT_2, 0x40)) {
      IDP_ERR << "Could not send command to actuator board" << std::endl;
      throw idp::Robot::ActuatorError();
    }
  }
  actuator1_is_on = true;
}

void idp::Robot::actuator1_off() {
  if (actuator2_is_on) {
    if (!_rlink.command(WRITE_PORT_2, 0x80)) {
      IDP_ERR << "Could not send command to actuator board" << std::endl;
      throw idp::Robot::ActuatorError();
    }
  }
  else {
    if (!_rlink.command(WRITE_PORT_2, 0xC0)) {
      IDP_ERR << "Could not send command to actuator board" << std::endl;
      throw idp::Robot::ActuatorError();
    }
  }
  actuator1_is_on = false;
}

void idp::Robot::actuator2_on() {
  if (actuator1_is_on) {
    if (!_rlink.command(WRITE_PORT_2, 0x00)) {
      IDP_ERR << "Could not send command to actuator board" << std::endl;
      throw idp::Robot::ActuatorError();
    }
  }
  else {
    if (!_rlink.command(WRITE_PORT_2, 0x80)) {
      IDP_ERR << "Could not send command to actuator board" << std::endl;
      throw idp::Robot::ActuatorError();
    }
  }
  actuator2_is_on = true;
}

void idp::Robot::actuator2_off() {
  if (actuator1_is_on) {
    if (!_rlink.command(WRITE_PORT_2, 0x40)) {
      IDP_ERR << "Could not send command to actuator board" << std::endl;
      throw idp::Robot::ActuatorError();
    }
  }
  else {
    if (!_rlink.command(WRITE_PORT_2, 0xC0)) {
      IDP_ERR << "Could not send command to actuator board" << std::endl;
      throw idp::Robot::ActuatorError();
    }
  }
  actuator2_is_on = false;
}

void idp::Robot::crack() {
  actuator1_off();
  actuator1_on();
}

void idp::Robot::release_claws() {
  actuator1_off();
}

void idp::Robot::eject() {
  actuator2_off();
  actuator2_on();
  delay(200);
  actuator2_off();
}

void idp::Robot::claws_open() {
  if (claws_are_open) {
    IDP_WARN << "Trying to open claws, but claws are already open." << std::endl;
  }
  else {
    int current_ms = _sw.read();
    _rlink.command(MOTOR_4_GO, 127);
    while (_sw.read() - current_ms < _constants.claw_open_time) {
      // Do nothing
    }
    _rlink.command(MOTOR_4_GO, 0);
    claws_are_open = true;
  }
}

void idp::Robot::claws_close() {
  if (!claws_are_open) {
    IDP_WARN << "Trying to close claws, but claws are already closed." << std::endl;
  }
  else {
    int current_ms = _sw.read();
    _rlink.command(MOTOR_4_GO, -127);
    while (_sw.read() - current_ms < _constants.claw_open_time) {
      // Do nothing
    }
    _rlink.command(MOTOR_4_GO, 0);
    claws_are_open = false;
  }
}

bool idp::Robot::egg_is_fake() {
  update_light_sensors();
  if (_light_sensors_history->back().value.values.egg_sensor == 0) // Light sensor board output is low
    return true;
  else return false;
}

bool idp::Robot::content_is_white() {
  update_light_sensors();
  if (_light_sensors_history->back().value.values.content_sensor == 1) // Light sensor board output is low
    return true;
  else return false;
}

