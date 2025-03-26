#include <MoveUtils.cpp>
#include <IMU.h>
#include <Calibration.cpp>
#include <pthread.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <sys/types.h>
#include <sys/wait.h>

int main(){ //todo setup pipes between threads.

    pthread_t ptid_signals,ptid_IMU,ptid_logic; 
    pthread_create(&ptid_signals, NULL, &startSystem, NULL);
    pthread_create(&ptid_IMU, NULL, &initialize, NULL);
    pthread_create(&ptid_logic, NULL, &startCalibration, NULL);

    int status;
    wait(&status);
    wait(&status);
    wait(&status);
}