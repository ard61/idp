# Thanks to http://mrbook.org/blog/tutorials/make/

CC=g++
ARM_CC=arm-linux-gnueabi-g++

CFLAGS=-c -ansi -Wall -g -DBOOST_LOG_DYN_LINK -I/export/teach/1BRobot
ARM_CFLAGS=-c -ansi -Wall -g -DBOOST_LOG_DYN_LINK -I/export/teach/1BRobot -I/usr/arm-unknown-linux-gnueabi/include

LDFLAGS=-L/export/teach/1BRobot -lrobot -lboost_program_options
ARM_LDFLAGS=-L/usr/arm-unknown-linux-gnueabi/lib -lrobot -lboost_program_options

SOURCES=main.cc robot.cc map.cc robot.h logging.h

OBJECTS=main.o robot.o map.o
ARM_OBJECTS=main_arm.o robot_arm.o map_arm.o

EXECUTABLE=robot
ARM_EXECUTABLE=robot.arm

all: $(SOURCES) $(EXECUTABLE) $(ARM_EXECUTABLE)

clean:
	rm -f *.o
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

$(ARM_EXECUTABLE): $(ARM_OBJECTS)
	$(ARM_CC) $(ARM_OBJECTS) -o $@ $(ARM_LDFLAGS)

main.o: main.cc robot.h logging.h
	$(CC) $(CFLAGS) main.cc -o $@

robot.o: robot.cc robot.h logging.h
	$(CC) $(CFLAGS) robot.cc -o $@

map.o: map.cc robot.h logging.h
	$(CC) $(CFLAGS) map.cc -o $@

main_arm.o: main.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) main.cc -o $@

robot_arm.o: robot.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) robot.cc -o $@

map_arm.o: map.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) map.cc -o $@
