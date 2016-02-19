#include <fstream>
#include <iostream>
#include <iomanip>
#include <exception>

#include <boost/program_options.hpp>

#include "robot.h"

const char idp::Robot::CONFIG_FILE[] = "robot.cfg";

idp::Robot::Robot() {
  _sw.start();
}

void idp::Robot::load_constants(int argc, char* argv[]) {
  /*
  Retrieves values for constants from command line and from configuration file
  using the boost/program_options library
  */

  namespace po = boost::program_options;
  po::options_description config("Configuration");

  config.add_options()
    ("robot_num", po::value<int>(&ROBOT_NUM)->default_value(9), "Wireless interface identifier for our robot")
    ("num_tests", po::value<int>(&NUM_TESTS)->default_value(100), "Number of tests in each test command")
    ("half_axle_length", po::value<double>(&HALF_AXLE_LENGTH)->default_value(10.0), "half-axle length")
    ("max_speed", po::value<double>(&MAX_SPEED)->default_value(0.2), "maximum speed of robot")
    ("map_file", po::value<std::string>(&MAP_FILE)->default_value("playing_area.map"), "playing araea map filename")
    ;
  
  po::variables_map vm;

  po::store(po::command_line_parser(argc, argv).options(config).run(), vm);
  po::notify(vm);

  std::ifstream ifs(CONFIG_FILE);
  if (!ifs)
  {
      std::cout << std::setw(10) << "WARNING: " << "Could not open config file: " << CONFIG_FILE << " ." << std::endl;
      std::cout << std::setw(10) << "WARNING: " << "Default values will be used for all parameters." << std::endl;
  }

  else
  {
      po::store(parse_config_file(ifs, config), vm);
      po::notify(vm);
  }
}

int idp::Robot::initialise_connection() {
  #ifndef __arm__
  int rc = _rlink.initialise(ROBOT_NUM);
  #else
  int rc = _rlink.initialise();
  #endif

  if (!rc) { // setup the link
    std::cout << std::setw(10) << "ERROR: " << "Cannot initialise link." << std::endl;
    _rlink.print_errs("    ");
    return -1;
  }
  return 0;
}

int idp::Robot::test() {
  stopwatch sw;
  int val;
  
  sw.start();
  for (int i = 0; i < NUM_TESTS; i++) {
    val = _rlink.request(TEST_INSTRUCTION);   // send test instruction
  }
  int etime = sw.stop();

  if (val == TEST_INSTRUCTION_RESULT) {     // check result
    std::cout << std::setw(10) << "INFO: " << "Test passed, " << "each test took on average " 
              << (double) etime/NUM_TESTS << " milliseconds." << std::endl;
    return 0;
  }
  
  else {
    throw idp::LinkError();
  }
  
  return -1;
}

int idp::Robot::move(MotorDemand motor_demand) {
  int motor1_demand = 128 * motor_demand.speed_l / idp::Robot::MAX_SPEED;
  int motor2_demand = 128 * motor_demand.speed_r / idp::Robot::MAX_SPEED;

  unsigned char speed1, speed2;
  if (motor1_demand > 0) speed1 = motor1_demand;
  else speed1 = 128 - motor1_demand;

  if (motor2_demand > 0) speed2 = motor2_demand;
  else speed2 = 128 - motor2_demand;
  
  if (!_rlink.command(MOTOR_1_GO, speed1)
      or !_rlink.command(MOTOR_2_GO, speed2)) {
    std::cout << std::setw(10) << "ERROR: " << "Error occurred in sending command to motor. " << std::endl;
  }
}

int idp::Robot::move(MotorDemand motor_demand, double distance) { 

}

int idp::Robot::turn(double angle) {

}

void idp::Robot::update_line_sensors() {

}


idp::MotorDemand idp::Robot::calculate_demand() {
  idp::MotorDemand motor_demand;

  // This is supposed to be our "equilibrium" state.  
  motor_demand.speed_l = _target_speed * (1 - HALF_AXLE_LENGTH * _target_curvature);
  motor_demand.speed_r = _target_speed * (1 + HALF_AXLE_LENGTH * _target_curvature);

  // Do some additionnal processing, based on the white line following strategy elaborated by Alistair.

  return motor_demand;
}

