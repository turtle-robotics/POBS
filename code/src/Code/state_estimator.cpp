#include "state_estimator.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

//TODO: Implement
StateEstimator::StateEstimator(){
    cur_state = new State;
    goal_state = new State;
    kalman = new KalmanFilter("src/Json/extended_kalman.json");
    node = new StateNode(kalman,goal_state);
    motorcontroller = new MotorController(goal_state,cur_state);
    
}

//Takes in data saves the state as the function runs
void StateEstimator::testFromFile(std::string filename){
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        std::cerr << "testFromFile: could not open " << filename << std::endl;
        return;
    }

    // Derive output filename alongside input (e.g. generatedTest.dat -> generatedTest_results.csv)
    std::string outName = filename;
    size_t dot = outName.rfind('.');
    if (dot != std::string::npos)
        outName = outName.substr(0, dot);
    outName += "_results.csv";

    std::ofstream outFile(outName);
    if (!outFile.is_open()) {
        std::cerr << "testFromFile: could not open output " << outName << std::endl;
        return;
    }

    outFile << "step,sensor,pos_x,pos_y,pos_z,ori_l,ori_i,ori_j,ori_k,vel_a,vel_b,vel_c\n";

    // Skip header until sensor data section
    std::string line;
    while (std::getline(inFile, line)) {
        if (line.find("--- SENSOR DATA ---") != std::string::npos)
            break;
    }

    int step = 0;
    auto writeState = [&](const std::string& sensor) {
        State s = kalman->estimateState();
        outFile << step++ << ","
                << sensor << ","
                << std::fixed << std::setprecision(6)
                << s.position.x << "," << s.position.y << "," << s.position.z << ","
                << s.orientation.l << "," << s.orientation.i << ","
                << s.orientation.j << "," << s.orientation.k << ","
                << s.velocites.a << "," << s.velocites.b << "," << s.velocites.c << "\n";
    };

    while (std::getline(inFile, line)) {
        if (line.find("IMU:") == 0) {
            // Parse: IMU:   [  ax,   ay,   az,   wx,   wy,   wz]
            double ax=0, ay=0, az=0, wx=0, wy=0, wz=0;
            std::istringstream ss(line.substr(line.find('[') + 1));
            char comma;
            ss >> ax >> comma >> ay >> comma >> az
               >> comma >> wx >> comma >> wy >> comma >> wz;
            Vector3<double> accel{ax, ay, az};
            Vector3<double> gyro {wx, wy, wz};
            kalman->measurementIMU(accel, gyro, 0.01);
            writeState("IMU");
        } else if (line.find("DEPTH:") == 0) {
            // Parse: DEPTH: [ d]
            double d = 0;
            std::istringstream ss(line.substr(line.find('[') + 1));
            ss >> d;
            kalman->measurementDepth(d);
            writeState("DEPTH");
        } else if (line.find("GPS:") == 0) {
            // Parse: GPS:   [  x,   y]
            double x = 0, y = 0;
            std::istringstream ss(line.substr(line.find('[') + 1));
            char comma;
            ss >> x >> comma >> y;
            Position gps{x, y, 0.0};
            kalman->measurementGPSPosition(gps);
            writeState("GPS");
        }
    }

    std::cout << "testFromFile: wrote " << step << " states to " << outName << std::endl;
}

//TODO: Implement
void StateEstimator::loadParams(std::string filename){

}    

//TODO: Implement
void StateEstimator::resetPosition(){

}

