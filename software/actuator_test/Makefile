all : actuator_test actuator_test.arm

actuator_test : actuator_test.cc 
	g++ -ansi -Wall -g -I/export/teach/1BRobot -L/export/teach/1BRobot actuator_test.cc -o actuator_test -lrobot

actuator_test.arm : actuator_test.cc
	arm-linux-gnueabi-g++ -ansi -Wall -g -I/usr/arm-unknown-linux-gnueabi/include -I/export/teach/1BRobot -L/usr/arm-unknown-linux-gnueabi/lib actuator_test.cc -o actuator_test.arm -lrobot
