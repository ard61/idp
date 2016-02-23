#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <string>
#include <vector>
#include <exception>
#include <cmath>

#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>


namespace idp {

/*
Physical variables use SI:
- lengths in m
- time in s
- angles in radians
*/


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

  Vector2d operator+ (const Vector2d &rhs) const { return Vector2d(x + rhs.x, y + rhs.y); }
  Vector2d operator* (const Vector2d &rhs) const { return Vector2d(x * rhs.x, y * rhs.y); }
  friend Vector2d operator* (const double &lhs, const Vector2d &rhs) { return Vector2d(lhs * rhs.x, lhs * rhs.y); }
  friend Vector2d operator* (const Vector2d &lhs, const double &rhs) { return Vector2d(lhs.x * rhs, lhs.y * rhs); }
  friend Vector2d operator/ (const Vector2d &lhs, const double &rhs) { return Vector2d(lhs.x / rhs, lhs.y / rhs); }

  double abs2() const {return x*x + y*y; }
  double abs() const {return sqrt(abs2()); }

  static Vector2d from_polar(double r, double theta) {return Vector2d(r * cos(theta), r * sin(theta)); }

  double x;
  double y;
}; // class Vector2d


class MotorDemand {
  // Contains the speed of each of the two wheels.  
public:
  MotorDemand() : speed_l(0.0), speed_r(0.0) {}
  MotorDemand(double &_speed_l, double &_speed_r) : speed_l(_speed_l), speed_r(_speed_r) {}

  double speed_l;
  double speed_r;
}; // class MotorDemand

template<typename T>
struct Timestamp
/*
This class contains a value of type T, as well as a timestamp.  
This timestamp (a time in milliseconds) is obviously intended to contain the time
when the value was last updated, although it can serve other functions.
*/
{
  T value;
  int ms;
};

class Robot {
public:
  Robot();
  ~Robot();

  // Initialisation
  void load_constants(int argc, char* argv[]);
  int initialise();
  int test();

  // Propulsion
  int move(MotorDemand &motor_demand);
  int move(MotorDemand &motor_demand, double &distance);
  int turn(double &angle);
  void update_tracking();

  // Position tracking
  struct Tracking {
    double speed;
    double curvature;  // Curvature is K = (angular velocity) / speed
    Vector2d position;
    double orientation;  // Orientation of the robot, in radians (anticlockwise from horizontal)
  };
  typedef std::vector<Timestamp<Tracking> > TrackingHistory;

  // Line following
  union LineSensors {
    struct values { // 0 means white, 1 means black
      unsigned int fore_l : 1;
      unsigned int fore_c : 1;
      unsigned int fore_r : 1;
      unsigned int rear_c : 1;
    };
    unsigned int cat;
  }; // union LineSensors

  void update_line_sensors();
  MotorDemand calculate_demand();
  bool is_lost();
  bool at_crossroad();

  // Egg processing
  void pickup_egg();
  int identify_egg();
  void crack_egg();
  int identify_inside();
  void drop_inside();
  void drop_shells();
  

private:
  // Config file name
  static const char config_file[];

  // Stopwatch
  stopwatch _sw;

  // Robot interface
  robot_link _rlink;

  // Constants, loaded from config file / command line
  struct Constants {
    int robot_num;
    int num_tests;
    double half_axle_length; // in m
    double max_speed; // in m/s
    std::string map_file;
    int ramp_time;
    double initial_position_x;
    double initial_position_y;
    double initial_orientation;
    double curve_curvature;
    double cruise_speed;
  } _constants;
  // Line following
  // Sensor values
  std::vector<LineSensors> _line_sensors;
  // Targets
  double _target_curvature; // in m^(-1), positive if turning to the left (anticlockwise when going forward)
  double _target_speed; // in m/s
  // Indicators
  bool _on_line;

  // Velocity and position tracking:
  TrackingHistory *_tracking_history;  // Allocated on heap

  /*
  Not sure about this yet.
  Timestamp<double> _position_tolerance_axial; // Axial tolerance is reset to white line width every time we reach a junction
  Timestamp<double> _position_tolerance_tangential; // Tangential tolerance is white line width while we are on a line
  */
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