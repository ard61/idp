all : motor_test motor_test.arm

motor_test : motor_test.cc 
	g++ -ansi -Wall -g -I/export/teach/1BRobot -L/export/teach/1BRobot motor_test.cc -o motor_test -lrobot

motor_test.arm : motor_test.cc
	arm-linux-gnueabi-g++ -ansi -Wall -g -I/usr/arm-unknown-linux-gnueabi/include -I/export/teach/1BRobot -L/usr/arm-unknown-linux-gnueabi/lib motor_test.cc -o motor_test.arm -lrobot