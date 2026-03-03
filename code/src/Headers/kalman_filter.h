#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "common_types.h"

class KalmanFilter {
public:
    KalmanFilter();

    bool init();
    void loadParams();

    void measurementGPS(const Position& gps_data);
    void measurementPressure(double pressure);
    void measurementIMU(const Orientation& imu_data);

    Position estimatePosition();

private:
    FilterParams filter_params;
    State estimated_state;
};

struct FilterParams {
};


#endif