#include <iostream>
#include "../src/IMU.cpp"

int main(){
    initialize((void*));

    std::cout<<read()[0]<<std::endl;

}