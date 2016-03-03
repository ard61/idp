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


class Vector2d {
public:
  Vector2d() : x(0.0), y(0.0) {}
  Vector2d(double _x, double _y) : x(_x), y(_y) {}

  Vector2d operator+ (const Vector2d &rhs) const { return Vector2d(x + rhs.x, y + rhs.y); }
  Vector2d operator- (const Vector2d &rhs) const { return Vector2d(x - rhs.x, y - rhs.y); }

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


class Point {
public:
  char name[3];
  Vector2d position;
  bool is_intersection;
}; // class Point


class Line {
public:
  char point1[3];
  char point2[3];

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
  int populate(const char* map_file);
  double distance_from_intersection(Vector2d position) const;

  std::vector<Point> points;
  std::vector<Line> lines;
}; // class Map


class PIDControlLoop {
public:
  PIDControlLoop() : _k_p(0.0), _k_i(0.0), _k_d(0.0), _derivative_smoothing_coef(0.0), 
                     _ms(0), _error(0), _derror(0), _ierror(0) {}
  void initialise(const double k_p, const double k_i, const double k_d, const double derivative_smoothing_coef);
  void update(const Timestamp<double> error);
  void clear() {
    _ms = 0;
    _error = 0.0;
    _derror = 0.0;
    _ierror = 0.0;
  }
  double get_demand() const;

private:
  double _k_p;
  double _k_i;
  double _k_d;

  double _derivative_smoothing_coef;

  int _ms;
  double _error;
  double _derror;
  double _ierror;
};


class Robot {
/*
Light sensor board: I2C bus, address 101.
  Pin 1-3: address 
  Pin 4-7: Light sensors 1-4
  Pin 8: ground
  Pin 9-10: LED
  Pin 11: LED (light sensor)
  Pin 12: LED (light sensor)
  Pin 13: Interrupt
  Pin 14: SCL
  Pin 15: SDA
  Pin 16: 5V

  Pin 1-3: address
  Pin 4-10: ground
  Pin 11: actuator circuit
  Pin 12: actuator circuit
  Pin 13 – 16: Interrupt / SCL / SDA / 5V
*/
public:
  Robot();
  ~Robot();

  // Exceptions
  class LinkError : public std::exception {
    virtual const char* what() const throw()
    {
      return "Error communicating with robot.";
    }
  };

  class LineFollowingError : public std::exception {
    virtual const char* what() const throw()
    {
      return "Line following algorithm is lost.";
    }  
  };

  class PositionTrackingError : public std::exception {
    virtual const char* what() const throw()
    {
      return "Position tracking does not match light sensor information.";
    }  
  };
  
  // Constants
  struct Constants {
    int robot_num;
    int num_tests;
    double half_axle_length; // in m
    double max_speed_l; // in m/s
    double max_speed_r;
    std::string map_file;
    int ramp_time;
    double initial_position_x;
    double initial_position_y;
    double initial_orientation;
    double curve_curvature;
    double cruise_speed;
    double control_loop_kp;
    double control_loop_ki;
    double control_loop_kd;
    double control_loop_derivative_smoothing_coef;
    double intersection_threshold_distance;
  } _constants;

  // Initialisation
  void load_constants();
  bool initialise();
  bool reinitialise();
  void configure();
  bool test();

  // Propulsion
  class MotorDemand {
    // Contains the speed of each of the two wheels.  
  public:
    MotorDemand() : speed_l(0.0), speed_r(0.0) {}
    MotorDemand(const double &_speed_l, const double &_speed_r) : speed_l(_speed_l), speed_r(_speed_r) {}

    double speed_l;
    double speed_r;
  }; // class MotorDemand

  void move(MotorDemand motor_demand);
  void turn(const double angle, const double angular_velocity);
  void turn(const double angle);

  // Position tracking
  void update_tracking();

  struct Tracking {
    double speed;
    double curvature;  // Curvature is K = (angular velocity) / speed
    Vector2d position;
    double orientation;  // Orientation of the robot, in radians (anticlockwise from horizontal)
  };
  typedef std::vector<Timestamp<Tracking> > TrackingHistory;

  // Line following
  union LightSensors {
    struct values { // 0 means white, 1 means black
      unsigned int front_left : 1;
      unsigned int front_center : 1;
      unsigned int front_right : 1;
      unsigned int rear_right : 1;
      unsigned int _padding : 2;
      unsigned int egg_sensor : 1;
      unsigned int content_sensor : 1;
    };
    unsigned int cat;
  }; // union LightSensors

  typedef std::vector<Timestamp<LightSensors> > LightSensorsHistory;

  void update_light_sensors();
  void line_sensor_analysis();
  void tracking_analysis();
  MotorDemand calculate_demand();

  // Recovery
  void recovery();

  // Egg processing
  void pickup_egg();
  int identify_egg();
  void crack_egg();
  int identify_inside();
  void drop_inside();
  void drop_shells();
  

private:
  // Stopwatch
  stopwatch _sw;

  // Robot interface
  robot_link _rlink;

  // Line following
  // Sensor values
  LightSensorsHistory *_light_sensors_history;  // Will be dynamically allocated
  double _target_curvature; // in m^(-1), positive if turning to the left (anticlockwise when going forward)
  double _target_speed;
  bool at_intersection;
  PIDControlLoop _control_loop;

  // Velocity and position tracking:
  TrackingHistory *_tracking_history;  // Allocated on heap

  // Map
  Map *_map;
  // Extra position tracking from map.  
  int current_line; // index of the line if we are on a line, but -1 if we are at an intersection.  
  int previous_point;
  bool at_point;
}; // class Robot

} // namespace idp

#endif
