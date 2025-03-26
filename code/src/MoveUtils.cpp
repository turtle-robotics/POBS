#include <PGPIO.h>
#include <Calibration.cpp>
#include <Hardware.h>
#include <string>
#include <cstddef>

struct Motor motors[4];
struct Servo servos[4];
SubProfile profiles;
int pi=0;

struct Motor{
    char* name;
    unsigned char flags;// 0,0,0,0,0,front,left side
    int gpio;
    int microseconds;
    double roll_torque_mult;
    double pitch_torque_mult;

    Motor(char* name, int gpio,unsigned char flags):name(name),gpio(gpio),flags(flags) {}
};

struct Servo{
    char* name;
    int gpio;
    double radians;
    unsigned char flags;// 0,0,0,0,0,front,left side

    Servo(char* name, int gpio,unsigned char flags):name(name),gpio(gpio),flags(flags) {}
};

void* startSystem(void * args){
    pi=pigpio_start(nullptr,nullptr);

    motors[0]=Motor("Front Left",Front_Left_Motor_GPIO,0b11);
    motors[1]=Motor("Front Right",Front_Right_Motor_GPIO,0b10);
    motors[2]=Motor("Back Left",Back_Left_Motor_GPIO,0b1);
    motors[3]=Motor("Back Right",Back_Right_Motor_GPIO,0b0);

    for(int n=0;n<4;n++){
        set_PWM_frequency(pi, motors[n].gpio, 50);
        motors[n].microseconds=0;
    }

    servos[0]=Servo("Front Left",Front_Left_Servo_GPIO,0b11);
    servos[1]=Servo("Front Right",Front_Right_Servo_GPIO,0b10);
    servos[2]=Servo("Back Left",Back_Left_Servo_GPIO,0b1);
    servos[3]=Servo("Back Right",Back_Right_Servo_GPIO,0b0);
}

double umsToDuty(int ums){
    return ums/20000.0;
}

bool rollLeft(double rotations_per_minute){
    // sets a target roll rate. counterclockwise
    double torque=Estimated_Rotational_Inertia_Roll/rotations_per_minute/4.0;
    for(int n=0;n<4;n++){
        double force=torque*motors[n].roll_torque_mult;
        double current_force = profiles.forward_thrust_profile_from_microseconds.evaluate(motors[n].microseconds);
        if(motors[n].flags & 0b1 == 0b1){
            int microseconds = profiles.forward_thrust_profile_from_force.evaluate(current_force-force);
            set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
            motors[n].microseconds=microseconds;
        }
        if(motors[n].flags & 0b0 == 0b0){
            int microseconds = profiles.forward_thrust_profile_from_force.evaluate(current_force+force);
            set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
            motors[n].microseconds=microseconds;
        }
    }
}  

bool rollRight(double rotations_per_minute){
    // sets a target roll rate.
    double torque=Estimated_Rotational_Inertia_Roll/rotations_per_minute/4.0;
    for(int n=0;n<4;n++){
        double force=torque*motors[n].roll_torque_mult;
        double current_force = profiles.forward_thrust_profile_from_microseconds.evaluate(motors[n].microseconds);
        if(motors[n].flags & 0b1 == 0b1){
            int microseconds = profiles.forward_thrust_profile_from_force.evaluate(current_force+force);
            set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
            motors[n].microseconds=microseconds;
        }
        if(motors[n].flags & 0b0 == 0b0){
            int microseconds = profiles.forward_thrust_profile_from_force.evaluate(current_force-force);
            set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
            motors[n].microseconds=microseconds;
        }
    }
}

bool pitchDown(double rotations_per_minute){
    // sets a target pitch rate.
    double torque=Estimated_Rotational_Inertia_Pitch/rotations_per_minute/4.0;
    for(int n=0;n<4;n++){
        double force=torque*motors[n].pitch_torque_mult;
        double current_force = profiles.forward_thrust_profile_from_microseconds.evaluate(motors[n].microseconds);
        if(motors[n].flags & 0b10 == 0b10){
            int microseconds = profiles.forward_thrust_profile_from_force.evaluate(current_force-force);
            set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
            motors[n].microseconds=microseconds;
        }
        if(motors[n].flags & 0b00 == 0b00){
            int microseconds = profiles.forward_thrust_profile_from_force.evaluate(current_force+force);
            set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
            motors[n].microseconds=microseconds;
        }
    }
}

bool pitchUp(double rotations_per_minute){
    // sets a target pitch rate.
    double torque=Estimated_Rotational_Inertia_Pitch/rotations_per_minute/4.0;
    for(int n=0;n<4;n++){
        double force=torque*motors[n].pitch_torque_mult;
        double current_force = profiles.forward_thrust_profile_from_microseconds.evaluate(motors[n].microseconds);
        if(motors[n].flags & 0b10 == 0b10){
            int microseconds = profiles.forward_thrust_profile_from_force.evaluate(current_force+force);
            set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
            motors[n].microseconds=microseconds;
        }
        if(motors[n].flags & 0b00 == 0b00){
            int microseconds = profiles.forward_thrust_profile_from_force.evaluate(current_force-force);
            set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
            motors[n].microseconds=microseconds;
        }
    }
}

bool setLinearAcceleration(double kilometers_per_hour){
    // negative values will go in reverse
    double force=Sub_Weight_KG * kilometers_per_hour/4.0;
    int microseconds;
    double duty = umsToDuty(microseconds);
    for(int n=0;n<4;n++){
        set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
        motors[n].microseconds=microseconds;
    }
}

bool changeVerticalLinearAcceleration(double kilometers_per_hour){
    // negative values will go in reverse
    double force=Sub_Weight_KG * kilometers_per_hour/4.0;
    int microseconds;
    if(kilometers_per_hour>=0){
        microseconds=profiles.forward_thrust_profile_from_force.evaluate(force);
    }
    else{
        microseconds=profiles.reverse_thrust_profile_from_force.evaluate(force);
    }
    double duty = umsToDuty(microseconds);
    for(int n=0;n<4;n++){
        set_servo_pulsewidth(pi, motors[n].gpio, microseconds);
        motors[n].microseconds=microseconds;
    }
}

void shutdownSystem(){
    pigpio_stop(pi);
}

