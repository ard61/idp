#include <iostream>
#include <unistd.h>

using namespace std;

#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>

#define ROBOT_NUM  9  // See number on our microcontroller

robot_link rlink;

int main ()
{
  int val;  // returned data from microprocessor

  #ifndef __arm__
  int rc = rlink.initialise(ROBOT_NUM);
  #else
  int rc = rlink.initialise();
  #endif

  if (!rc) {      // setup the link
    cout << "Cannot initialise link" << endl;
    rlink.print_errs("    ");
    return -1;
  }

  stopwatch sw;
  const int num_tests = 1000;

  sw.start();
  for (int i = 0; i < num_tests; i++) {
    val = rlink.request (TEST_INSTRUCTION);   // send test instruction
  }
  sw.stop();

  if (val == TEST_INSTRUCTION_RESULT) {     // check result
    cout << "Test passed" << endl;
    cout << "Each test took on average " << sw.read()/num_tests << " seconds." << endl;
    return 0;                             // all OK, finish
  }
  
  else if (val == REQUEST_ERROR) {
    cout << "Fatal errors on link:" << endl;
    rlink.print_errs();
  }
  
  else cout << "Test failed (bad value returned)" << endl;

  return -1;
}