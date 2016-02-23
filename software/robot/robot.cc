#include <fstream>
#include <iostream>
#include <iomanip>
#include <exception>

#include <boost/program_options.hpp>

#include "robot.h"

const char idp::Robot::config_file[] = "robot.cfg";


idp::Robot::Robot() {
  _sw.start();

  _tracking_history = new TrackingHistory();
}

idp::Robot::~Robot() {
  _sw.stop();
  delete _tracking_history;
}

void idp::Robot::load_constants(int argc, char* argv[]) {
  /*
  Retrieves values for constants from command line and from configuration file
  using the boost/program_options library
  */

  namespace po = boost::program_options;
  po::options_description config("Configuration");

  config.add_options()
    ("robot_num", po::value<int>(&(_constants.robot_num))->default_value(9), "Wireless interface identifier for our robot")
    ("num_tests", po::value<int>(&(_constants.num_tests))->default_value(100), "Number of tests in each test command")
    ("half_axle_length", po::value<double>(&(_constants.half_axle_length))->default_value(10.0), "half-axle length")
    ("max_speed", po::value<double>(&(_constants.max_speed))->default_value(0.2), "maximum speed of robot")
    ("map_file", po::value<std::string>(&(_constants.map_file))->default_value("playing_area.map"), "playing area map filename")
    ("ramp_time", po::value<int>(&(_constants.ramp_time))->default_value(255), "Ramp time for robot motors")
    ("initial_position_x", po::value<double>(&(_constants.initial_position_x))->default_value(0.1), "Robot initial position, x-coordinate")
    ("initial_position_y", po::value<double>(&(_constants.initial_position_y))->default_value(0.1), "Robot initial position, y-coordinate")
    ("initial_orientation", po::value<double>(&(_constants.initial_orientation))->default_value(0), "Robot initial orientation")
    ("curve_curvature", po::value<double>(&(_constants.curve_curvature))->default_value(1.67), "Curvature of the curved white line paths")
    ("cruise_speed", po::value<double>(&(_constants.cruise_speed))->default_value(0.03), "Standard speed of the robot")
    ;
  
  po::variables_map vm;

  po::store(po::command_line_parser(argc, argv).options(config).run(), vm);
  po::notify(vm);

  std::ifstream ifs(config_file);
  if (!ifs)
  {
      std::cout << std::setw(10) << "WARNING: " << "Could not open config file: " << config_file << " ." << std::endl;
      std::cout << std::setw(10) << "WARNING: " << "Default values will be used for all parameters." << std::endl;
  }

  else
  {
      po::store(parse_config_file(ifs, config), vm);
      po::notify(vm);
  }

  // Put the first value of position and speed in the _tracking_history data structure
  idp::Timestamp<idp::Robot::Tracking> tracking;
  tracking.ms = _sw.read();
  tracking.value.speed = 0;
  tracking.value.curvature = 0;
  tracking.value.position = idp::Vector2d(_constants.initial_position_x, _constants.initial_position_y);
  tracking.value.orientation = _constants.initial_orientation;

  _tracking_history->push_back(tracking);

}

int idp::Robot::initialise() {
  #ifndef __arm__
  int rc = _rlink.initialise(_constants.robot_num);
  #else
  int rc = _rlink.initialise();
  #endif

  if (!rc) { // setup the link
    std::cout << std::setw(10) << "ERROR: " << "Cannot initialise link." << std::endl;
    _rlink.print_errs("    ");
    return -1;
  }

  // Set motor ramp time
  _rlink.command(RAMP_TIME, _constants.ramp_time);
  return 0;
}

int idp::Robot::test() {
  stopwatch sw;
  int val;
  
  sw.start();
  for (int i = 0; i < _constants.num_tests; i++) {
    val = _rlink.request(TEST_INSTRUCTION);   // send test instruction
  }
  int etime = sw.stop();

  if (val == TEST_INSTRUCTION_RESULT) {     // check result
    std::cout << std::setw(10) << "INFO: " << "Test passed, " << "each test took on average " 
              << (double) etime/_constants.num_tests << " milliseconds." << std::endl;
    return 0;
  }
  
  else {
    std::cout << std::setw(10) << "ERROR: " << "An error occured during preliminary test." << std::endl;
    throw idp::LinkError();
  }
  
  return -1;
}

int idp::Robot::move(MotorDemand &motor_demand) {
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
    std::cout << std::setw(10) << "ERROR: " << "Error occurred in sending command to motor. " << std::endl;
    throw idp::LinkError();
  }

  // Update current speed and update position estimate


  return 0;
}

int idp::Robot::move(MotorDemand &motor_demand, double &distance) { 

}

int idp::Robot::turn(double &angle) {

}

void idp::Robot::update_tracking() {
  // Get velocity information from motors
  int speed_1 = _rlink.request(MOTOR_1);
  int speed_2 = _rlink.request(MOTOR_2);

  if (speed_1 == REQUEST_ERROR or speed_2 == REQUEST_ERROR) {
    std::cout << std::setw(10) << "ERROR: " << "Error occurred in requesting speed information from motors. " << std::endl;
    throw idp::LinkError();
  }
  if (speed_1 > 127) speed_1 = 128 - speed_1;
  if (speed_2 > 127) speed_2 = 128 - speed_2;

  // TODO: Adjust this when final robot orientation is known
  speed_1 = -speed_1;

  double speed_l = idp::Robot::max_speed / 128 * speed_1;
  double speed_r = idp::Robot::max_speed / 128 * speed_2;

  // Calculate new speed, curvature, position and orientation using second-order Euler integrator
  idp::Timestamp<idp::Robot::Tracking> tracking;
  tracking.ms = _sw.read();
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

void idp::Robot::update_line_sensors() {

}


idp::MotorDemand idp::Robot::calculate_demand() {
  idp::MotorDemand motor_demand;



  // This is supposed to be our "equilibrium" state.  
  motor_demand.speed_l = _target_speed * (1 - _constants.half_axle_length * _target_curvature);
  motor_demand.speed_r = _target_speed * (1 + _constants.half_axle_length * _target_curvature);

  // Do some additionnal processing, based on the white line following strategy elaborated by Alistair.

  return motor_demand;
}

