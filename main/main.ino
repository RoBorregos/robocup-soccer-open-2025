#include <PWMServo.h>  
#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
#include "Photo.h"
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
bool pixy_seen = false;
const int BUFFER_SIZE = 50;
char buffer[BUFFER_SIZE];
const int servo_min = 1000;
const int servo_mid = 1500;
const int servo_max = 2000;
float time_shoot = 2000;


BNO055 bno;
PWMServo dribbler;
Photo photo;
PID pid(0.65, 0.01 , 0.6, 200);


Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);


void setup() {
  Serial.begin(115200);
  dribbler.attach(6);
  Serial1.begin(115200);
  motors.InitializeMotors();
  bno.InitializeBNO();
}


double radians_degrees(double radians) {
  return radians * (180.0 / M_PI);
}


void loop() {
  bno.GetBNOData();
  double current_yaw = bno.GetYaw();


  if (Serial1.available()){
    int bytesRead = Serial1.readBytesUntil('\n', buffer, sizeof(buffer));
    buffer[bytesRead] = '\0';
    char* token = strtok(buffer, " ");
    ball_distance = atof(token);


    token = strtok(NULL, " ");
    ball_angle = atof(token);


    token = strtok(NULL, " ");
    goal_angle = atof(token);


    token = strtok(NULL, " ");
    goal_distance = atof(token);


    open_ball_seen = (ball_distance != 0 || ball_angle != 0);
  }

  photo.ReadPhotoBack
  photo.ReadPhotoFront
  photo.ReadPhotoLeft
  photo.ReadPhotoRight

  double error = bno.analize_error(setpoint,current_yaw);
  double speed_w = pid.Calculate(setpoint, error); //Checar si esta bien asi o hay que invertir los valores y aplicar la logica para los diversos casos
  double speed_goal = 155;
  double speed_ball = 200;

  //Checar si vale la pena hacer un switch case para los fototransistores
  
  if (photo.PhotoBack()){
    motors.MoveMotorsImu(0, 200, speed_w); 
    delay(200); 
  }

  if (photo.PhotoFront()){
    motors.MoveMotorsImu(180, 200, speed_w);
    delay(200);
  }

  if (photo.PhotoLeft()){
    motors.MoveMotorsImu(90, 200, speed_w);
    delay(200);
  }

  if (photo.PhotoRight()){
    motors.MoveMotorsImu(270, 200, speed_w);
    delay(200);
  }
  
 
  if (speed_w != 0) {
    if (open_ball_seen){
      double error_ball = bno.analize_error(ball_angle, current_yaw);
      double differential_ball = error_ball * 0.001; //Calcular el error diferecial
      ponderated_ball = ball_angle + differential_ball;
      motors.MoveMotorsImu(ponderated_ball, abs(speed_ball), speed_w);
      dribbler.writeMicroseconds(servo_mid);
      if (goal_angle != 0) {
      double error_goal = bno.analize_error(goal_angle, current_yaw);
      double differential_goal = error_goal * 0.001; //Calcular el error diferencial por medio de prueba y error
      ponderated_goal = goal_angle + differential_goal;
      motors.MoveMotorsImu(ponderated_goal, abs(speed_goal), speed_w);
      } else {
        motors.StopMotors();
        digitalWrite(KICKER_PIN, HIGH);
        delay(20);
        digitalWrite(KICKER_PIN, LOW);
        dribbler.writeMicroseconds(servo_min);
      }
    }
  

    if (!open_ball_seen) {
      start = millis();
      if (goal_angle != 0 ) {
        double error_goal = bno.analize_error(goal_angle, current_yaw);
        double differential_goal = error_goal * 0.001; //Calcular el error diferencial por medio de prueba y error
        ponderated_goal = goal_angle + differential_goal;
        motors.MoveMotorsImu(ponderated_goal, abs(speed_goal), speed_w);
        dribbler.writeMicroseconds(servo_mid);
        if (goal_distance > 20){ //Checar si se puede medir la distancia con la OpenMV
          motors.StopMotors();
          digitalWrite(KICKER_PIN, HIGH);
          delay(20);
          digitalWrite(KICKER_PIN, LOW);
        }
      } else {
        motors.StopMotors();
        digitalWrite(KICKER_PIN, HIGH);
        delay(20);
        digitalWrite(KICKER_PIN, LOW);
      }
      if (time_shoot > start) {
          motors.StopMotors();
          dribbler.writeMicroseconds(servo_min);
      }
    }
  }
}



