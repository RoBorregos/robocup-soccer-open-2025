#ifndef constantes_h
#define constantes_h
#include <stdint.h>



const int MOTOR3_IN1 = 34; 
const int MOTOR3_IN2 = 33;
const int MOTOR3_PWM = 4; 

const int MOTOR4_IN1 = 30;  // 
const int MOTOR4_IN2 = 31;  // 
const int MOTOR4_PWM = 3;  // 

const int MOTOR2_IN1 = 36;  // 
const int MOTOR2_IN2 = 35;  // 
const int MOTOR2_PWM = 5;  // 

const int MOTOR1_IN1 = 29;  // 
const int MOTOR1_IN2 = 28;  // 
const int MOTOR1_PWM = 2;  //

const int KICKER_PIN =  32;
const int DRIBBLER_PIN = 6;

const int frontPins[] = {};
const int backPins[] = {};
const int rightPins[] = {};
const int leftPins[] = {};

const int front_array = 1;
const int back_array = 2;
const int right_array = 3;
const int left_array = 4;
//Poner el ping del Kicker

/*
-----------------Espacio para los fototransistores-----------------
*/


//Threshold for line values
const int lineThreshold_left = 2500;
const int lineThreshold_right = 2500;
const int lineThreshold_front = 2500;
const int lineThreshold_back = 2500;

#endif  