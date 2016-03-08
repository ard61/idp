#include <iostream>

#include <robot_link.h>
#include <robot_instr.h>

#include <delay.h>

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
    std::cout << "Cannot initialise link." << std::endl;
    return -1;
  }
  
  delay(2000);
    
  std::cout << "Turning actuator 1 on." << std::endl;
  rlink.command(WRITE_PORT_2, 0x40);
  
  delay(2000);
  
  std::cout << "Turning actuator 1 off." << std::endl;
  rlink.command(WRITE_PORT_2, 0xC0);
  
  delay(2000);
  
  std::cout << "Turning actuator 2 on." << std::endl;
  rlink.command(WRITE_PORT_2, 0x80);
  
  delay(2000);
  
  std::cout << "Turning actuator 2 off." << std::endl;
  rlink.command(WRITE_PORT_2, 0xC0);
  
  return 0;  
}
