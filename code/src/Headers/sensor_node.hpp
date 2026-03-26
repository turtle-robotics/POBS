#ifndef SENSOR_NODE_HPP
#define SENSOR_NODE_HPP

#include "common_types.hpp"
#include <string>

//Use this to store the needed params for the code
struct SensorParams {
};

//This node reads the data from the sensors and publishes that to the ros network
class SensorNode {
public:
    //Starts up the ros node and begins publishing
    SensorNode(std::string file_name);

    //Helper function to allow manual reading from a json
    bool loadParams(std::string file_name);
    //Saves camera to a file
    void writeCamera();
    //Saves sonar to a file to a file
    void writeSonar();

    //Takes in the reading from the depth sensore I2C
    double readPressure();
    //Takes in the reading from the IMU
    Orientation readIMU();

    //This what runs when a state is recived stores the state
    void stateCallback(const State& incoming_state);
    
private:
    SensorParams params;
    State state;
    IMUData last_imu;
    double last_pressure;
};



#endif