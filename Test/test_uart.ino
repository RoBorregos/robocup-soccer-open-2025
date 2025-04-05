#include <Servo.h>  
#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
#include <cmath>


//Incluir la libreria del servo para el Kicker
//#include "Photo.h"


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
float dribbler_distance = 0;
float dribbler_angle = 0;
float goal_angle = 0;
float ball_distance = 0;
float last_distance = 0;
float last_angle = 0;
float distance = 0;
float goal_distance = 0;
float distance_pixels = 0;
float differential_ball = 0;
float differential_goal = 0;
float differential_dribbler = 0;
bool open_ball_seen = false;
bool dribbler_ball_seen = false;
bool goal_seen = false;
bool ball_captured = false;
const int BUFFER_SIZE = 50;
char buffer1[BUFFER_SIZE];
char buffer2[BUFFER_SIZE];
const int servo_min = 1000;
const int servo_mid = 1500;
const int servo_max = 2000;
int time_shoot = 2000;

BNO055 bno;
Servo servo;


PID pid(0.65, 0.01 , 0.6, 200);


Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  delay(1000);
}


void loop() { 
  if (Serial1.available()){
    Serial.println("Reading data from OpenMV 1...");
    int bytesRead = Serial1.readBytesUntil('\n', buffer1, sizeof(buffer1));
    buffer1[bytesRead] = '\0';

    Serial.print("Raw Data 1: ");
    Serial.println(buffer1);

    char* token = strtok(buffer1, " ");
    ball_distance = atof(token);



    token = strtok(NULL, " ");
    ball_angle = atof(token);


    token = strtok(NULL, " ");
    goal_angle = atof(token);


    token = strtok(NULL, " ");
    goal_distance = atof(token);


    open_ball_seen = (ball_distance != 0 && ball_angle != 0);
    goal_seen = (goal_angle != 0 && goal_distance != 0);
    delay(50);

  } else {
    Serial.println("NO se detecta camara 1");
  }
/*
  if (Serial2.available()){
    Serial.println("Reading data from OpenMV 2...");
    int bytesRead = Serial2.readBytesUntil('\n', buffer2, sizeof(buffer2));
    buffer2[bytesRead] = '\0';

    Serial.print("Raw Data 2: ");
    Serial.println(buffer2);

    char* token = strtok(buffer2, " ");
    dribbler_distance = atof(token);



    token = strtok(NULL, " ");
    dribbler_angle = atof(token);


    token = strtok(NULL, " ");
    goal_angle = atof(token);


    token = strtok(NULL, " ");
    goal_distance = atof(token);


    dribbler_ball_seen = (dribbler_distance != 0 && dribbler_angle != 0);
    ball_captured = (dribbler_distance <= 40 && dribbler_angle == 0);
    delay(50);
  }
  */
}