#ifndef SENSOR_UTILS_HPP
#define SENSOR_UTILS_HPP


#include "common_types.hpp"
#include <string>
#define FreshwaterDensity  1000.0    // kg/m^3 (freshwater)
#define SaltwaterDensity  1025.0     // kg/m^3 (saltwater)
#define G  9.81         // m/s^2

//Returns m
double pressureToDepth(double pressure,double atmospheric_pressure, bool is_freshwater){ 
    double gauge_pressure = pressure - atmospheric_pressure;

    // Prevent negative depth if pressure is below atmospheric
    if (gauge_pressure <= 0.0) {
        return 0.0;
    }

    if(is_freshwater)
        return gauge_pressure / (FreshwaterDensity * G);

    return gauge_pressure / (SaltwaterDensity * G);
}







#endif