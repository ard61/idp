#include <fstream>

#include <boost/log/trivial.hpp>
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
    ("HALF_AXLE_LENGTH", po::value<double>(&HALF_AXLE_LENGTH)->default_value(10.0), "half-axle length")
    ("max_speed", po::value<double>(&MAX_SPEED)->default_value(0.2), "maximum speed of robot")
    ("map_file", po::value<std::string>(&MAP_FILE)->default_value("playing_area.map"), "playing araea map filename")
    ;
  
  po::variables_map vm;

  po::store(po::command_line_parser(argc, argv).options(config).run(), vm);
  po::notify(vm);

  std::ifstream ifs(CONFIG_FILE);
  if (!ifs)
  {
      BOOST_LOG_TRIVIAL(warning) << "Could not open config file: " << CONFIG_FILE << " .";
      BOOST_LOG_TRIVIAL(warning) << "Default values will be used for all parameters.";
  }

  else
  {
      po::store(parse_config_file(ifs, config), vm);
      po::notify(vm);
  }
}

idp::MotorDemand idp::Robot::calculate_demand() {
  idp::MotorDemand motor_demand;

  // This is supposed to be our "equilibrium" state.  
  motor_demand.speed_l = _target_speed * (1 - HALF_AXLE_LENGTH * _target_curvature);
  motor_demand.speed_r = _target_speed * (1 + HALF_AXLE_LENGTH * _target_curvature);

  // Do some additionnal processing, based on the white line following strategy elaborated by Alistair.

  return motor_demand;
}

int idp::Robot::move(MotorDemand motor_demand) {
  int motor1_demand = 128 * motor_demand.speed_l / idp::Robot::MAX_SPEED;
  int motor2_demand = 128 * motor_demand.speed_r / idp::Robot::MAX_SPEED;
  
  if (!_rlink.command(MOTOR_1_GO, motor1_demand)
      or !_rlink.command(MOTOR_2_GO, motor2_demand)) {
    BOOST_LOG_TRIVIAL(error) << "Error occurred in sending command to motor. ";
  }
}

int idp::Robot::move(MotorDemand motor_demand, double distance) { 

}

int idp::Robot::turn(double angle) {

}