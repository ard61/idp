# Thanks to http://mrbook.org/blog/tutorials/make/

CC=g++
ARM_CC=arm-linux-gnueabi-g++

CFLAGS=-c -Wall -g -DBOOST_LOG_DYN_LINK -I/export/teach/1BRobot
ARM_CFLAGS=-c -Wall -g -DBOOST_LOG_DYN_LINK -I/export/teach/1BRobot -I/usr/arm-unknown-linux-gnueabi/include

LDFLAGS=-L/export/teach/1BRobot -lrobot -lboost_program_options -lpthread -lboost_log
ARM_LDFLAGS=-L/usr/arm-unknown-linux-gnueabi/lib -L/export/teach/1BRobot -lrobot.arm -lboost_program_options -lpthread -lboost_log

SOURCES=main.cc robot.cc map.cc

OBJECTS=main.o robot.o map.o
ARM_OBJECTS=main.arm.o robot.arm.o map.arm.o

EXECUTABLE=robot
ARM_EXECUTABLE=robot.arm

all: $(SOURCES) $(EXECUTABLE) $(ARM_EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

$(ARM_EXECUTABLE): $(ARM_OBJECTS)
	$(ARM_CC) $(ARM_OBJECTS) -o $@ $(ARM_LDFLAGS)

main.o:
	$(CC) $(CFLAGS) main.cc -o $@

robot.o:
	$(CC) $(CFLAGS) robot.cc -o $@

map.o:
	$(CC) $(CFLAGS) map.cc -o $@

main.arm.o:
	$(ARM_CC) $(ARM_CFLAGS) main.cc -o $@

robot.arm.o:
	$(ARM_CC) $(ARM_CFLAGS) robot.cc -o $@

map.arm.o:
	$(ARM_CC) $(ARM_CFLAGS) map.cc -o $@
