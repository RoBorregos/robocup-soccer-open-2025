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

const int KICKER_PIN =  32; //Poner el ping del Kicker
const int DRIBBLER_PIN = 6; //Poner el pin del Dribbler

/*
-----------------Espacio para los fototransistores-----------------
*/


//Threshold for line values
const int lineThreshold_left = 2500;
const int lineThreshold_right = 2500;
const int lineThreshold_front = 2500;
const int lineThreshold_back = 2500;

// Receive signals
const uint8_t RECEIVE_BNO = 's';
const uint8_t RECEIVE_BALL_ANGLE= 'c';
const uint8_t RECEIVE_BALL_DISTANCE = 'd';
const uint8_t RECEIVE_GOAL_ANGLE = 'g';
const uint8_t RECEIVE_GOAL_DISTANCE = 'k';
const uint8_t RECEIVE_LINE_ANGLE = 'r';

#endif  