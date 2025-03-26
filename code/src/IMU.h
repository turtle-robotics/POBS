#ifndef IMU_H
#define IMU_H

#include <unistd.h>

template <typename T>
struct vec3{
    T a;
    T b;
    T c;
};

//setup a pipe between this thread and main.
int pipes[2];
int i2c_fd;
const char *i2c_device = "/dev/i2c-1";
int slave_address = 0x50;


void* initialize(void* args);


#endif

