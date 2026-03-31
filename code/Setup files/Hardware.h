#ifndef HARDWARE_H
#define HARDWARE_H


/*----------- These are the hardware pinouts -----------*/
#define Front_Left_Motor_GPIO 5
#define Front_Right_Motor_GPIO 6
#define Back_Left_Motor_GPIO 13
#define Back_Right_Motor_GPIO 16

#define Front_Left_Servo_GPIO 23
#define Front_Right_Servo_GPIO 24
#define Back_Left_Servo_GPIO 25
#define Back_Right_Servo_GPIO 26

/*----------- Important parameters to update ----------*/
#define Sub_Volume_Cubic_Meters 0
#define Sub_Weight_KG 0
#define Estimated_Rotational_Inertia_Roll 0
#define Estimated_Rotational_Inertia_Pitch 0
#define Estimated_Rotational_Inertia_Yaw 0

//This will scale all values in the system to be lower to prevent over drawing.
#define Max_Forward_Thrust_ums 1800
#define Max_Reverse_Thrust_ums 1200

/*----------- Motor dependant information ----------*/
#define Forward_Thrust_Profile_From_Force -6.483843,101.1,1510 //The thrust profiles as a quadratic from force to pwm in ms
#define Reverse_Thrust_Profile_From_Force 9.8379,-126.7698,1490


#define Forward_Thrust_Profile_From_Microseconds .00003779, -.1113, 81.94 
#define Reverse_Thrust_Profile_From_Microseconds -.00002448, .07673, -59.95


#endif // MY_HEADER_H