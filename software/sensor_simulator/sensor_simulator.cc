#include <iostream>

#include <robot_link.h>
#include <robot_instr.h>

int main(int argc, char* argv[]) {
  robot_link rlink;
  int rc;

  #ifdef __arm__
  rc = rlink.initialise();
  #else
  const int ROBOT_NUM = 9;
  rc = rlink.initialise(ROBOT_NUM);
  #endif

  if (!rc) {
    std::cout << "Cannot initialise link" << std::endl;
    return -1;
  }

  return 0;  
}
