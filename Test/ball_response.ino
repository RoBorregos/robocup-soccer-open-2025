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
bool goal_seen = false;
bool pixy_seen = false;
const int BUFFER_SIZE = 50;
char buffer[BUFFER_SIZE];
const int servo_min = 1000;
const int servo_mid = 1500;
const int servo_max = 2000;
int time_shoot = 2000;


BNO055 bno;
Servo dribbler;
//Photo photo;


PID pid(4, 0.01 , 0.6, 500); //0.6, 0.01, 0.6, 200


Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  dribbler.attach(6);
  dribbler.writeMicroseconds(servo_min);
  motors.InitializeMotors();
  bno.InitializeBNO();
  delay(1000);
}


void loop() { 
  bno.GetBNOData();
  double current_yaw = bno.GetYaw();


  if (Serial1.available()){
    Serial.println("Reading data from OpenMV...");
    int bytesRead = Serial1.readBytesUntil('\n', buffer, sizeof(buffer));
    buffer[bytesRead] = '\0';

    Serial.print("Raw Data: ");
    Serial.println(buffer);

    char* token = strtok(buffer, " ");
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

  }

  double error = bno.analize_error(setpoint,current_yaw);
  double speed_w = pid.Calculate(setpoint, error); //Checar si esta bien asi o hay que invertir los valores y aplicar la logica para los diversos casos
  double speed_goal = 155;
  double speed_ball = 100;
 

  if (open_ball_seen){
      double error_ball = ball_angle + current_yaw;
      double differential_ball = error_ball * 0.001; //Calcular el error diferecial
      ponderated_ball = -(ball_angle + differential_ball);
      motors.MoveMotorsImu(ponderated_ball, abs(speed_ball), speed_w);
      dribbler.writeMicroseconds(servo_mid);
  } else {
      motors.MoveMotorsImu(0,0,speed_w);
      if (speed_w == 0) {
        motors.SetAllSpeeds(80);
        motors.MoveBackward();
      }
    }
}

