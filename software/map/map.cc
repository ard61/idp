#include <fstream>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <string>

#include "map.h"

Map::Map(){
  std::strcpy("playing_area.map", map_filename);
}

void Map::populate() {
  /*  Function that parses the .map file containing nicely human-readable
      information about point and line positioning on the playing area.
      This function contains absolutely horrible code with c-string handling.
      Don't attempt to understand or change it.  Just don't.
      Oh, and it's terribly inefficient too.  I just can't be bothered to look
      at this horrible code anymore.  It hurts my brain.  
  */

  std::ifstream map_file;
  map_file.open(map_filename);

  // Variables used inside loop
  char cur_line[16];
  int line_num = 1;
  bool are_dealing_with_points = true;
  Point current_point;
  Line current_line;

  while (map_file.good()) {
    if (line_num = 1 && (std::string(cur_line) != "POINTS:\n")) {
      // Sanity check here
      std::cout << "Invalid map file specified.";
    }

    else if (std::string(cur_line) == "LINES:\n") are_dealing_with_points = false;

    else if (are_dealing_with_points) {
      // Using the *HORRIBLE* std::strtok function, which is probably
      // one of the reasons why nobody uses C anymore these days. 
      strcpy(std::strtok(cur_line, " \n"), current_point.name);

      current_point.coord_x = std::atoi(std::strtok(NULL, " \n"));

      current_point.coord_y = std::atoi(std::strtok(NULL, " \n"));

      // Append to member variable
      _points.push_back(current_point);
    }

    else { // We're dealing with lines
      strcpy(std::strtok(cur_line, " \n"), current_line.point1);

      strcpy(std::strtok(NULL, " \n"), current_line.point2);

      switch (*(std::strtok(NULL, " \n"))) {
      case 'S':
        current_line.is_straight = true;
      case 'C':
        current_line.is_straight = false;
      }

      switch (*(std::strtok(NULL, " \n"))) {
      case 'V':
        current_line.orientation = 0;
      case 'H':
        current_line.orientation = 1;
      case 'P':
        current_line.orientation = 2;
      case 'N':
        current_line.orientation = 3;
      }

      // Append to member variable
      _lines.push_back(current_line);
    }
  }
  map_file.close();

  // WHEW!
}

int main(int argc, char* argv[]) {
  // This function exists only to make stuff compile.
  return 0;
}