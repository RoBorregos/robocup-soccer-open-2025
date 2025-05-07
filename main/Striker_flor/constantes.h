#ifndef constantes_h
#define constantes_h
#include <stdint.h>



const int MOTOR3_IN1 = 34; //34
const int MOTOR3_IN2 = 33; //33
const int MOTOR3_PWM = 4; 

const int MOTOR4_IN1 = 30;  // 30
const int MOTOR4_IN2 = 31;  // 31
const int MOTOR4_PWM = 3;  // 

const int MOTOR2_IN1 = 35;  // 36
const int MOTOR2_IN2 = 36;  // 35
const int MOTOR2_PWM = 5;  // 

const int MOTOR1_IN1 = 28;  // 29
const int MOTOR1_IN2 = 29;  // 28
const int MOTOR1_PWM = 2;  //

const int KICKER_PIN =  32; //Ping del Kicker

//Constantes para velocidades del dribbler
const int servo_min = 1000;
const int servo_mid = 1300;
const int servo_max = 1600;

const uint8_t selectPins[3] = {16, 15, 14};  // S0, S1, S2 Pines de seleccion
const uint8_t muxPins[4] = {A6, A7, A9, A14}; // Pines de salida MUX

// Receive signals
const uint8_t RECEIVE_BNO = 's';
const uint8_t RECEIVE_BALL_ANGLE= 'c';
const uint8_t RECEIVE_BALL_DISTANCE = 'd';
const uint8_t RECEIVE_GOAL_ANGLE = 'g';
const uint8_t RECEIVE_GOAL_DISTANCE = 'k';
const uint8_t RECEIVE_LINE_ANGLE = 'r';


#endif  
