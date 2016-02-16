#include <iostream>
#include <iomanip>

#include <robot_link.h>
#include <robot_instr.h>

#include <delay.h>

int main(int argc, char* argv[]) {
  robot_link rlink;
  int rc;
  const int DELAY_MSECS = 10;
  const request_instruction instruction_array[8] = {READ_PORT_0, READ_PORT_1, READ_PORT_2, READ_PORT_3, READ_PORT_4, READ_PORT_5, READ_PORT_6, READ_PORT_7};

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

  int I2C_readings[8];
  const int I2C_display_width = 6;
  
  while (!rlink.fatal_err())
  {
    std::cout << "Listening on I2C ports 0-7. " << std::endl;
    
    for (int i = 0; i < 8; i++) {
      rc = rlink.request(instruction_array[i]);
      
      if (rc != REQUEST_ERROR) {
        I2C_readings[i] = rc;
      }

      else {
        std::cout << "Error occured while requesting I2C port " 
                  << i << " . Exiting." << std::endl;
        return -1;
      }
    }

    for (int i = 0; i < 8; i++) {
      std::cout << std::setw(I2C_display_width) << I2C_readings[i];
    }

    std::cout << std::endl;

    delay(DELAY_MSECS);
  }

  rlink.print_errs();

  return 0;  
}
