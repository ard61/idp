#include <iostream>

#include <robot_link.h>
#include <robot_instr.h>

#include <delay.h>

int main(int argc, char* argv[]) {
  robot_link rlink;
  int rc;
  const int DELAY_MSECS = 5000;
  const command_instruction instruction_array[8] = {MOTOR_1_GO, MOTOR_2_GO, MOTOR_3_GO, MOTOR_4_GO};

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
  
  while (!rlink.fatal_err())
  {
    for (int i = 0; i < 4; i++) {

    std::cout << "Testing motor " << i+1 << ". " << std::endl;
      rlink.command(instruction_array[i], 127);

      for (int j = 0; j < 4; j++) {
        if (j != i) rlink.command(instruction_array[j], 0);
      }

    delay(DELAY_MSECS);
    }
  }

  rlink.print_errs();

  return 0;  
}
