#include <fstream>
#include <iostream>

#include "logging.h"
#include "robot.h"

void idp::PIDControlLoop::initialise(const double k_p, const double k_i, const double k_d, const double derivative_smoothing_coef) {
  _k_p = k_p;
  _k_i = k_i;
  _k_d = k_d;
  _derivative_smoothing_coef = derivative_smoothing_coef;
}

void idp::PIDControlLoop::update(const idp::Timestamp<double> error) {
  if (_ms == 0) { // This is the first value input to control loop. 
    _error = error.value;
    _ierror = error.value;
    _ms = error.ms;
    return;
  }

  // Otherwise: 
  int dt = error.ms - _ms;
  _ierror += error.value * dt;
  _derror = _derivative_smoothing_coef * (error.value - _error) / dt + (1 - _derivative_smoothing_coef) * _derror;
  _error = error.value;
  _ms = error.ms;
}

double idp::PIDControlLoop::get_demand() const {
  return _k_p * _error + _k_i * _ierror + _k_d * _derror;
}

idp::Robot::Robot() {
  _sw.start();

  _light_sensors_history = new LightSensorsHistory();
  _tracking_history = new TrackingHistory();
  _map = new idp::Map();
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
  _constants.half_axle_length = 10.0;  // half-axle length
  _constants.max_speed = 0.2;  // maximum speed of robot
  _constants.curve_curvature = 1.67;  // Curvature of the curved white line paths
  _constants.cruise_speed = 0.05;  // Standard speed of the robot
  _constants.ramp_time = 255;  // Ramp time for robot motors

  // Tracking
  _constants.initial_position_x = 0.1;  // Robot initial position, x-coordinate
  _constants.initial_position_y = 0.1;  // Robot initial position, y-coordinate
  _constants.initial_orientation = 0;  // Robot initial orientation

  // Line following
  _constants.control_loop_kp = 5;  // Proportional control loop coefficient
  _constants.control_loop_ki = 0;  // Integral control loop coefficient
  _constants.control_loop_kd = 0;  // Derivative control loop coefficient
  _constants.control_loop_derivative_smoothing_coef = 0.8;  // Control loop derivative smoothing coefficient
  _constants.intersection_threshold_distance = 0.05; // We are 'close' to an intersection if distance smaller than this value.


  // Put the first value of position and speed in the _tracking_history data structure
  idp::Timestamp<idp::Robot::Tracking> tracking;
  tracking.ms = _sw.read();
  tracking.value.speed = 0;
  tracking.value.curvature = 0;
  tracking.value.position = idp::Vector2d(_constants.initial_position_x, _constants.initial_position_y);
  tracking.value.orientation = _constants.initial_orientation;

  _tracking_history->push_back(tracking);

  _control_loop.initialise(_constants.control_loop_kp, _constants.control_loop_ki, 
                           _constants.control_loop_kd, _constants.control_loop_derivative_smoothing_coef);

  _map->populate(_constants.map_file.c_str());

  _target_curvature = 0;
  _target_speed = _constants.cruise_speed;
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
    throw idp::Robot::LinkError();
  }
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

void idp::Robot::move(const idp::Robot::MotorDemand &motor_demand) {
  // Preliminary bounds check
  if (motor_demand.speed_l > _constants.max_speed
      or motor_demand.speed_l < -_constants.max_speed) {
    IDP_WARN << "An error occured during preliminary test." << std::endl;
  }

  // TODO: Put negative signs here if needed when final motor orientation becomes known
  int motor1_demand = 128 * motor_demand.speed_l / _constants.max_speed;
  int motor2_demand = 128 * motor_demand.speed_r / _constants.max_speed;

  unsigned char speed_l, speed_r;
  if (motor1_demand > 0) speed_l = motor1_demand;
  else speed_l = 128 - motor1_demand;

  if (motor2_demand > 0) speed_r = motor2_demand;
  else speed_r = 128 - motor2_demand;

  /*
  Motor wiring:

  MOTOR_1 -> red -> left motor
  MOTOR_2 -> green -> right motor
  MOTOR_3 -> blue
  MOTOR_4 -> yellow
  */
  
  if (!_rlink.command(MOTOR_1_GO, speed_l)
      or !_rlink.command(MOTOR_2_GO, speed_r)) {
    IDP_ERR << "Error occurred in sending command to motor. " << std::endl;
    throw idp::Robot::LinkError();
  }
}

void idp::Robot::turn(const double angle, const double angular_velocity) {
  int delta_t_ms = angle / angular_velocity * 1000;

  double speed = angular_velocity * _constants.half_axle_length;

  // If angular velocity is positive (anticlockwise), then left motor should go backwards.  
  idp::Robot::MotorDemand motor_demand(-speed, speed);
  idp::Robot::MotorDemand zero(0,0);

  int current_ms = _sw.read();
  move(motor_demand);

  while (_sw.read() - current_ms < delta_t_ms - 10) { // 10 ms to account for lag
    // Do nothing
  }

  move(zero);
}


void idp::Robot::turn(const double angle) {
  turn(angle, _constants.cruise_speed / _constants.half_axle_length);
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

  double speed_l = idp::Robot::_constants.max_speed / 128 * speed_1;
  double speed_r = idp::Robot::_constants.max_speed / 128 * speed_2;

  // Calculate new speed, curvature, position and orientation using second-order Euler integrator
  tracking.value.speed = (speed_l + speed_r) / 2;
  tracking.value.curvature = (speed_r - speed_l) / (2 * _constants.half_axle_length * tracking.value.speed);

  double delta_t = tracking.ms - _tracking_history->back().ms;

  double prev_orientation = _tracking_history->back().value.orientation;
  double prev_curvature = _tracking_history->back().value.curvature;
  double prev_speed = _tracking_history->back().value.speed;
  idp::Vector2d prev_position = _tracking_history->back().value.position;

  tracking.value.orientation = prev_orientation + delta_t * ((tracking.value.speed + prev_speed) / 2) * ((tracking.value.curvature + prev_curvature) / 2);
  tracking.value.position = prev_position + delta_t * ((tracking.value.speed + prev_speed) / 2) * idp::Vector2d::from_polar(1, (tracking.value.orientation + prev_orientation) / 2);

  // Store into the _tracking data structure

  _tracking_history->push_back(tracking);
}

void idp::Robot::update_light_sensors() {
  idp::Timestamp<idp::Robot::LightSensors> light_sensors;
  int rc = _rlink.request(READ_PORT_5);

  if (rc == REQUEST_ERROR) {
    IDP_ERR << "Error occurred when reading from light sensor board (I2C address 5). " << std::endl;
    throw idp::Robot::LinkError();
  }

  light_sensors.ms = _sw.read();
  light_sensors.value.cat = rc;

  _light_sensors_history->push_back(light_sensors);
}


void idp::Robot::line_sensor_analysis() {

  int line_sensors = _light_sensors_history->back().value.cat bitand 0x0F;
  int ms = _light_sensors_history->back().ms;

  idp::Timestamp<double> x;

  bool close_to_intersection = (_map->distance_from_intersection(_tracking_history->back().value.position) > _constants.intersection_threshold_distance);

  switch (line_sensors bitand 0x07) { // ignore rear sensor now
    case 0x00:
      // All black! We are definitely lost here.  
      throw idp::Robot::LineFollowingError();
      break;

    case 0x01:
      // All black except the left sensor.  
      x.value = 2;
      x.ms = ms;
      _control_loop.update(x);
      break;

    case 0x02:
      // Center sensor is white, we are right on target.  
      x.value = 0;
      x.ms = ms;
      _control_loop.update(x);
      break;

    case 0x03:
      // Left and center sensors are white
      x.value = 1;
      x.ms = ms;
      _control_loop.update(x);
      break;

    case 0x04:
      // All black except the far right sensor.  
      x.value = -2;
      x.ms = ms;
      _control_loop.update(x);
      break;

    case 0x05:
      if (!close_to_intersection) {
        IDP_ERR << "Position tracking says we are far from intersection but we read value " 
                  << std::hex << line_sensors << " from light sensor board." << std::endl;
        throw idp::Robot::PositionTrackingError();
      }
      else {
        IDP_WARN << "Robot is at an odd angle to line, starting recovery procedure." << std::endl;
        throw idp::Robot::LineFollowingError();
      }
      break;

    case 0x06:
      // Right and center sensors are white
      x.value = -1;
      x.ms = ms;
      _control_loop.update(x);
      break;

    case 0x07:
      if (!close_to_intersection) {
        IDP_WARN << "Robot is at an odd angle to line, starting recovery procedure." << std::endl;
        throw idp::Robot::LineFollowingError();
      }
      else {
        // We hit that intersection!!!
        at_intersection = true;
        IDP_INFO << "We have reached the intersection. " << std::endl;
      }
      break;
  }
}

void idp::Robot::tracking_analysis() {
  // To be performed after line following tells us whether we have arrived at an intersection or not. 
  
}

idp::Robot::MotorDemand idp::Robot::calculate_demand() {
  idp::Robot::MotorDemand motor_demand;

  motor_demand.speed_l = _target_speed * (1 - _constants.half_axle_length * (_target_curvature + _control_loop.get_demand()));
  motor_demand.speed_r = _target_speed * (1 + _constants.half_axle_length * (_target_curvature + _control_loop.get_demand()));

  return motor_demand;
}

void idp::Robot::recovery() {
  // First we need to know which line we are on
}