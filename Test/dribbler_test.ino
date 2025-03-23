#include <PWMServo.h>  
#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
#include <cmath>


float bno_angle = 0;
unsigned long start_millis;
unsigned long current_millis;
unsigned long previous_millis = 0;
float setpoint = 0 ;
float translation_angle = 0;
float adjust_angle = 0;
float angle_degrees = 0;
float ponderated_ball = 0;
float ponderated_goal = 0;
float ball_angle = 0;
float goal_angle = 0;
float ball_distance = 0;
float last_distance = 0;
float last_angle = 0;
float distance = 0;
float goal_distance = 0;
float distance_pixels = 0;
float differential_ball = 0;
float differential_goal = 0;
bool open_ball_seen = false;
bool pixy_seen = false;
const int BUFFER_SIZE = 50;
char buffer[BUFFER_SIZE];
const int servo_min = 1000;
const int servo_mid = 1500;
const int servo_max = 2000;

PWMServo dribbler;

void setup() {
    Serial1.begin(115200);
    Serial.begin(115200);
    dribbler.attach(6);
    Serial1.begin(115200);
}

void loop() {
    dribbler.writeMicroseconds(servo_mid);
    delay(1000);
    dribbler.writeMicroseconds(servo_min);
    delay(1000);
    dribbler.writeMicroseconds(servo_max);
    delay(1000);
    dribbler.writeMicroseconds(0);
    delay(5000);
}