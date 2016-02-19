#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <string>
#include <vector>
#include <exception>

#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>


namespace idp {

class LinkError : public std::exception {
  virtual const char* what() const throw()
  {
    return "Error communicating with robot.";
  }
};

class Vector2d {
public:
  Vector2d() : x(0.0), y(0.0) {}
  Vector2d(double _x, double _y) : x(_x), y(_y) {}

  double x;
  double y;
}; // class Vector2d


union LineSensors {
  struct values {
    unsigned int fore_l : 1;
    unsigned int fore_c : 1;
    unsigned int fore_r : 1;
    unsigned int rear_c : 1;
  };
  unsigned int cat;
}; // union LineSensors


class MotorDemand {
  // Contains the speed of each of the two wheels.  
public:
  MotorDemand() : speed_l(0.0), speed_r(0.0) {}
  MotorDemand(double _speed_l, double _speed_r) : speed_l(_speed_l), speed_r(_speed_r) {}

  double speed_l;
  double speed_r;
}; // class MotorDemand


class Robot {
public:
  Robot();
  ~Robot();

  // Initialisation
  void load_constants(int argc, char* argv[]);
  int initialise_connection();
  int test();

  // Propulsion
  int move(MotorDemand motor_demand);
  int move(MotorDemand motor_demand, double distance);
  int turn(double angle);

  // Line following
  void update_line_sensors();
  MotorDemand calculate_demand();
  bool is_lost();
  bool at_crossroad();

private:
  // Config file name
  static const char CONFIG_FILE[];

  // Stopwatch
  stopwatch _sw;

  // Robot interface
  robot_link _rlink;

  // Constants, loaded from config file / command line
  int ROBOT_NUM;
  int NUM_TESTS;
  double HALF_AXLE_LENGTH; // in m
  double MAX_SPEED; // in m/s
  std::string MAP_FILE;

  // Line following
  // Sensor values
  std::vector<LineSensors> _line_sensors;
  // Targets
  double _target_curvature; // in m^(-1), positive if turning to the left (anticlockwise when going forward)
  double _target_speed; // in m/s

  // Tracking variables
  Vector2d _position;
  Vector2d _position_tolerance;
}; // class Robot


class Point {
public:
  char name[2];
  Vector2d position;
}; // class Point


class Line {
public:
  char point1[2];
  char point2[2];

  bool is_straight;  // true for a straight line, false for circular arc
  int orientation;
  /*  0 if the straight line is vertical, or the circular arc from point1 to point2 is vertical
      1 if horizontal
      2 if at 45 degrees anticlockwise from horizontal
      3 if at 45 degrees clockwise from horizontal
  */
}; // class Line


class Map {
public:
  Map();
  int populate(char* map_file);

private:
  std::vector<Point> _points;
  std::vector<Line> _lines;
}; // class Map


} // namespace idp

#endif