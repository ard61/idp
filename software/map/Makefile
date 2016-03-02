# Thanks to http://mrbook.org/blog/tutorials/make/

CC=g++
ARM_CC=arm-linux-gnueabi-g++

CFLAGS=-c -ansi -Wall -g -I/export/teach/1BRobot

LDFLAGS=-L/export/teach/1BRobot -lrobot

SOURCES= map.cc robot.h logging.h

OBJECTS=map.o

EXECUTABLE=map

all: $(SOURCES) $(EXECUTABLE)

clean:
	rm -f *.o
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

map.o: map.cc robot.h logging.h
	$(CC) $(CFLAGS) map.cc -o $@