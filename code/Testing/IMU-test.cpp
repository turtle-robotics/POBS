#include <iostream>
#include "../src/IMU.h"

int main(){
    IMU::initialize((void*)1);

    std::cout<<IMU::read()[0]<<std::endl;

}