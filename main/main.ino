#include <Servo.h>  
#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
#include "Photo.h"
#include <cmath>
//Incluir la libreria del servo para el Kicker


float bno_angle = 0;
unsigned long start_millis;
unsigned long current_millis;
unsigned long previous_millis = 0;
float cathethus = 0;
float setpoint = 0 ;
float translation_angle = 0;
float adjust_angle = 0;
float angle_degrees = 0;
float ponderated_ball = 0;
float ponderated_goal = 0;
float ponderated_dribbler = 0;
float ball_angle = 0;
float dribbler_distance = 0;
float dribbler_angle = 0;
float goal_angle = 0;
float ball_distance = 0;
float last_distance = 0;
float last_angle = 0;
float distance = 0;
float goal_distance = 0;
float own_angle = 0;
float own_distance = 0;
float distance_pixels = 0;
float differential_ball = 0;
float differential_goal = 0;
float differential_dribbler = 0;
bool open_ball_seen = false;
bool dribbler_ball_seen = false;
bool goal_seen = false;
bool ball_captured = false;
bool own_seen = false;
const int BUFFER_SIZE = 100;
char buffer1[BUFFER_SIZE];
char buffer2[BUFFER_SIZE];
const int servo_min = 1000;
const int servo_mid = 1700;
const int servo_max = 2000;
int time_shoot = 2000;
String serial1_line = "";
String serial2_line = "";
uint8_t front[2] = {A8, A9};
uint8_t right[4] = {A3, A12, A13, A14};
uint8_t left[4] = {A6, A15, A16, A17};
uint8_t back[4] = {A0, A1, A2, A7};


BNO055 bno;
Servo dribbler;
//Photo photo;


PID pid(6, 0.01 , 0.6, 600); //0.6, 0.01, 0.6, 200
PID pid2(3, 0.01 , 0.6, 500);


Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);
    
PhotoSensors sensors(front, left, right, back);

void setup() {
    Serial1.begin(115200);
    Serial.begin(115200);
    dribbler.attach(6);
    delay(1000);
}


void loop() {
    float = sensors.getAverage(FRONT);
    Serial.print("Valor de la foto: ");
    Serial.println(valor);
}

