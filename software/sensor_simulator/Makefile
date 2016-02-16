all : sensor_simulator sensor_simulator.arm

sensor_simulator : sensor_simulator.cc 
	 g++ -ansi -Wall -g -I/export/teach/1BRobot -L/export/teach/1BRobot sensor_simulator.cc -o sensor_simulator -lrobot

sensor_simulator.arm: sensor_simulator.cc
	arm-linux-gnueabi-g++ -ansi -Wall -g -I/usr/arm-unknown-linux-gnueabi/include -I/export/teach/1BRobot -L/usr/arm-unknown-linux-gnueabi/lib sensor_simulator.cc -o sensor_simulator.arm -lrobot
