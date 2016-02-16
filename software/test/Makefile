all : test test.arm

test : test.cc 
	g++ -ansi -Wall -g -I/export/teach/1BRobot -L/export/teach/1BRobot test.cc -o test -lrobot

test.arm : test.cc
	arm-linux-gnueabi-g++ -ansi -Wall -g -I/usr/arm-unknown-linux-gnueabi/include -I/export/teach/1BRobot -L/usr/arm-unknown-linux-gnueabi/lib test.cc -o test -lrobot