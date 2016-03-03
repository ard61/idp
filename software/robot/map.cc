#include <fstream>
#include <cstring>
#include <cstdlib>
#include <string>
#include <iostream>

#include "logging.h"
#include "robot.h"

idp::Map::Map() {

}

int idp::Map::populate(const char* map_file) {
  /*  Function that parses the .map file containing nicely human-readable
      information about point and line positioning on the playing area.
      This function contains absolutely horrible code with c-string handling.
      Don't attempt to understand or change it.  Just don't.
      Oh, and it's terribly inefficient too.  I just can't be bothered to look
      at this horrible code anymore.  It hurts my brain.  
  */

  std::ifstream ifs;
  ifs.open(map_file);

  // Variables used inside loop
  int line_number = 0;
  char cur_line[16];
  bool are_dealing_with_points = true;
  Point current_point;
  Line current_line;

  line_number++;
  ifs.getline(cur_line, 16);
  if (strncmp(cur_line, "POINTS:", 7) != 0) {
    // Sanity check here
    IDP_ERR << "Error parsing playing area map file on line 1." << std::endl;
    return -1;
  }

  while (ifs.good()) {
    line_number++;
    ifs.getline(cur_line, 16);

    if (strncmp(cur_line, "LINES:", 6) == 0) are_dealing_with_points = false;

    else if (are_dealing_with_points) {
      // Using the *HORRIBLE* std::strtok function, which is probably
      // one of the reasons why nobody uses C anymore these days. 
      // Basically you need to check if it returns NULL every time you
      // call it, otherwise nasty runtime pointer resolving errors can
      // occur.

      char* strtok_return;
      bool error = false;

      strtok_return = std::strtok(cur_line, " \n");
      if (strtok_return != NULL) {
        strncpy(current_point.name, strtok_return, 2);
        current_point.name[2] = '\0';
      }
      else { 
        error = true;
      }

      strtok_return = std::strtok(NULL, " \n");
      if (strtok_return != NULL) {
        current_point.position.x = std::atof(strtok_return);
      }
      else { 
        error = true;
      }

      strtok_return = std::strtok(NULL, " \n");
      if (strtok_return != NULL) {
        current_point.position.y = std::atof(strtok_return);
      }
      else { 
        error = true;
      }

      strtok_return = std::strtok(NULL, " \n");
      if (strtok_return != NULL && std::strncmp(strtok_return, "NI", 2) == 0) {
        current_point.is_intersection = false;
      }
      else if (strtok_return == NULL) {
        current_point.is_intersection = true;
      }
      else { 
        error = true;
      }

      if (error) {
        IDP_ERR << "Error parsing playing area map file on line " << line_number << "." << std::endl;
        return -1;
      }

      // Append to member variable
      points.push_back(current_point);
    }

    else { 
      // We're dealing with lines
      
      char* strtok_return;
      bool error = false;

      strtok_return = std::strtok(cur_line, " \n");
      if (strtok_return != NULL) {
        strncpy(current_line.point1, strtok_return, 2);
        current_line.point1[2] = '\0';
      }
      else error = true;

      strtok_return = std::strtok(NULL, " \n");
      if (strtok_return != NULL) {
        strncpy(current_line.point2, strtok_return, 2);
        current_line.point2[2] = '\0';
      }
      else error = true;

      strtok_return = std::strtok(NULL, " \n");
      if (strtok_return != NULL) {
        switch (*strtok_return) {
        case 'S':
          current_line.is_straight = true;
          break;
        case 'C':
          current_line.is_straight = false;
          break;
        }
      }
      else error = true;

      strtok_return = std::strtok(NULL, " \n");
      if (strtok_return != NULL) {
        switch (*strtok_return) {
        case 'V':
          current_line.orientation = 0;
          break;
        case 'H':
          current_line.orientation = 1;
          break;
        case 'P':
          current_line.orientation = 2;
          break;
        case 'N':
          current_line.orientation = 3;
          break;
        }
      }
      else error = true;

      if (error) {
        IDP_ERR << "Error parsing playing area map file on line " << line_number << "." << std::endl;
        return -1;
      }

      // Append to member variable
      lines.push_back(current_line);
    }
  }
  ifs.close();

  IDP_INFO << "Read " << points.size() << " points and "
            << lines.size() << " lines from playing area map file." << std::endl;
 
  // WHEW!
  return 0;
}

double idp::Map::distance_from_intersection(idp::Vector2d position) const {
  double final_distance = 10000; // Dummy large value to start with.  

  for (int i = 0; i < points.size(); i++) {
    double distance = (position - points[i].position).abs();
    if (points[i].is_intersection && distance < final_distance) {
      final_distance = distance;
    }
  }

  return final_distance;
}