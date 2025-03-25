#include <Servo.h>  
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
float goal_angle_180 = 0;
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
float defense_limit = 0;
const int distance_limit_min = 30;
const int distance_limit_max = 40;
bool defense_zone = false;

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
  dribbler.attach(DRIBBLER_PIN);
  Serial1.begin(115200);
  motors.InitializeMotors();
  bno.InitializeBNO();
  analogReadResolution(12);
  pinMode(KICKER_PIN, OUTPUT);
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
    goal_distance = atof(token);


    token = strtok(NULL, " ");
    goal_angle = atof(token);

    open_ball_seen = (ball_distance != 0 || ball_angle != 0);
  }

  if (goal_distance > distance_limit_min && goal_distance < distance_limit_max){
    defense_zone = true;
  } else {
    defense_zone = false;
  }

  photo.ReadPhotoBack
  photo.ReadPhotoFront
  photo.ReadPhotoLeft
  photo.ReadPhotoRight

  double error = bno.analize_error(setpoint,current_yaw);
  double speed_w = pid.Calculate(setpoint, error); //Checar si esta bien asi o hay que invertir los valores y aplicar la logica para los diversos casos
  double speed_goal = 155;
  double speed_ball = 200;

  defense_limit = ball_distance + goal_distance;


  if (open_ball_seen && defense_limit < 70 && defense_zone){ //Checar valores de distancia
    if (ball_angle != 0){
      double error_ball = ball_angle + current_yaw;
      double differential_ball = error_ball * 0.001; //Calcular el error diferecial
      ponderated_ball = ball_angle + differential_ball;
      motors.MoveMotorsImu(ponderated_ball, abs(speed_ball), speed_w);
    } else {
      double error_goal = goal_angle + current_yaw;
      double differential_goal = error_goal * 0.001; //Calcular el error diferencial por medio de prueba y error
      ponderated_goal = goal_angle + differential_goal;
      motors.MoveMotorsImu(ponderated_goal, abs(speed_goal), speed_w);
    }
  } else {
    motors.MoveMotorsImu(0, 0, speed_w);
    if (goal_angle != 180 && goal_angle >  0) {
      motors.MoveRight();
    } else if (goal_angle != 180 && goal_angle < 0) {
      motors.MoveLeft();
    } else if (goal_angle == 180) {
      motors.StopMotors();
    }
  }

}




