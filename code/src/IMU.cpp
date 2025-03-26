#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <IMU.h>

uint8_t* read(){
    uint8_t* buffer=new uint8_t[32]; // Adjust buffer size as needed
    int bytes_read;

    bytes_read = read(i2c_fd, buffer, sizeof(buffer));
    if (bytes_read < 0) {
        perror("Failed to read from I2C device");
        close(i2c_fd);
        return;
    } else if (bytes_read == 0) {
       printf("No data available\n");
    }
    else {
        printf("Received %d bytes: ", bytes_read);
        for (int i = 0; i < bytes_read; i++) {
            printf("%02X ", buffer[i]);
        }
        printf("\n");
    }
    return &buffer;
}

void* initialize(void* args){

    i2c_fd = open(i2c_device, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C device");
        return;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, slave_address) < 0) {
        perror("Failed to set I2C slave address");
        close(i2c_fd);
        return;
    }
   
}