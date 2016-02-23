#include <iostream>
#include <iomanip>

#include "robot.h"

int main(int argc, char* argv[]) {
  idp::Robot r;
  
  r.load_constants(argc, argv);

  if (!r.initialise()) {
    std::cout << std::setw(10) << "INFO: " << "Exiting. " << std::endl;
    return -1;
  }

  try {
    r.test();
  }

  catch (idp::LinkError& e) {
    std::cout << std::setw(10) << "INFO: " << "Exiting. " << std::endl;
    return -1;
  }

  // Main loop here.

  while (true) {
    
  }

  return 0;
}