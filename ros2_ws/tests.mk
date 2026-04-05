CC = gcc # Compiler to use
OPTIONS = -O2 -g -Wall # -g for debug, -O2 for optimise and -Wall additional messages
INCLUDES = -I . # Directory for header file
OBJS = test.o IMU.o # List of objects to be build
.PHONY: all clean # To declare all, clean are not files

all: ${OBJS}
	${CC} ${OPTIONS} ${OBJS} -o test_linked.exe
test.o: ./Testing/IMU-test.cpp
	${CC} ${OPTIONS} -I ./Testing -c ./Testing/IMU-test.cpp
IMU.o: ./src/IMU.cpp ./src/IMU.h
	${CC} ${OPTIONS} -I ./src -c ./src/IMU.cpp 
