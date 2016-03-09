# Thanks to http://mrbook.org/blog/tutorials/make/

CC=g++
ARM_CC=arm-linux-gnueabi-g++

CFLAGS=-c -ansi -Wall -g -I/export/teach/1BRobot
ARM_CFLAGS=-c -ansi -Wall -g -I/export/teach/1BRobot -I/usr/arm-unknown-linux-gnueabi/include

LDFLAGS=-L/export/teach/1BRobot -lrobot
ARM_LDFLAGS=-L/usr/arm-unknown-linux-gnueabi/lib -lrobot

SOURCES=main.cc robot.cc map.cc robot.h logging.h line_following_test.cc propulsion_test.cc actuator_test.cc led_test.cc claws_test.cc

OBJECTS=main.o robot.o map.o
ARM_OBJECTS=main_arm.o robot_arm.o map_arm.o

EXECUTABLE=robot
ARM_EXECUTABLE=robot.arm

all: $(SOURCES) $(EXECUTABLE) $(ARM_EXECUTABLE) line_following_test line_following_test.arm propulsion_test propulsion_test.arm actuator_test actuator_test.arm led_test led_test.arm claws_test claws_test.arm

clean:
	rm -f *.o
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

$(ARM_EXECUTABLE): $(ARM_OBJECTS)
	$(ARM_CC) $(ARM_OBJECTS) -o $@ $(ARM_LDFLAGS)

line_following_test: line_following_test.o robot.o map.o
	$(CC) line_following_test.o robot.o map.o -o $@ $(LDFLAGS)

line_following_test.arm: line_following_test_arm.o robot_arm.o map_arm.o
	$(ARM_CC) line_following_test_arm.o robot_arm.o map_arm.o -o $@ $(ARM_LDFLAGS)

propulsion_test: propulsion_test.o robot.o map.o
	$(CC) propulsion_test.o robot.o map.o -o $@ $(LDFLAGS)

propulsion_test.arm: propulsion_test_arm.o robot_arm.o map_arm.o
	$(ARM_CC) propulsion_test_arm.o robot_arm.o map_arm.o -o $@ $(ARM_LDFLAGS)

actuator_test: actuator_test.o robot.o map.o
	$(CC) actuator_test.o robot.o map.o -o $@ $(LDFLAGS)

actuator_test.arm: actuator_test_arm.o robot_arm.o map_arm.o
	$(ARM_CC) actuator_test_arm.o robot_arm.o map_arm.o -o $@ $(ARM_LDFLAGS)

led_test: led_test.o robot.o map.o
	$(CC) led_test.o robot.o map.o -o $@ $(LDFLAGS)

led_test.arm: led_test_arm.o robot_arm.o map_arm.o
	$(ARM_CC) led_test_arm.o robot_arm.o map_arm.o -o $@ $(ARM_LDFLAGS)
	
claws_test: claws_test.o robot.o map.o
	$(CC) claws_test.o robot.o map.o -o $@ $(LDFLAGS)

claws_test.arm: claws_test_arm.o robot_arm.o map_arm.o
	$(ARM_CC) claws_test_arm.o robot_arm.o map_arm.o -o $@ $(ARM_LDFLAGS)


main.o: main.cc robot.h logging.h
	$(CC) $(CFLAGS) main.cc -o $@

robot.o: robot.cc robot.h logging.h
	$(CC) $(CFLAGS) robot.cc -o $@

map.o: map.cc robot.h logging.h
	$(CC) $(CFLAGS) map.cc -o $@

line_following_test.o: line_following_test.cc robot.h logging.h
	$(CC) $(CFLAGS) line_following_test.cc -o $@

propulsion_test.o: propulsion_test.cc robot.h logging.h
	$(CC) $(CFLAGS) propulsion_test.cc -o $@

actuator_test.o: actuator_test.cc robot.h logging.h
	$(CC) $(CFLAGS) actuator_test.cc -o $@

led_test.o: led_test.cc robot.h logging.h
	$(CC) $(CFLAGS) led_test.cc -o $@

claws_test.o: claws_test.cc robot.h logging.h
	$(CC) $(CFLAGS) claws_test.cc -o $@


main_arm.o: main.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) main.cc -o $@

robot_arm.o: robot.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) robot.cc -o $@

map_arm.o: map.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) map.cc -o $@

line_following_test_arm.o: line_following_test.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) line_following_test.cc -o $@

propulsion_test_arm.o: propulsion_test.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) propulsion_test.cc -o $@
	
actuator_test_arm.o: actuator_test.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) actuator_test.cc -o $@

led_test_arm.o: led_test.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) led_test.cc -o $@

claws_test_arm.o: claws_test.cc robot.h logging.h
	$(ARM_CC) $(ARM_CFLAGS) claws_test.cc -o $@
