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
float opp_goal_angle = 0;
float ball_distance = 0;
float last_distance = 0;
float last_angle = 0;
float distance = 0;
float goal_distance = 0;
float opp_goal_distance = 0;
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
int time_shoot = 2000;
int speed_goal = 155;
int speed_ball = 100;




BNO055 bno;
Servo servo;
//Photo photo;




PID pid(1.5, 0.00735 , 45, 200);




Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);




void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  motors.InitializeMotors();
  bno.InitializeBNO();


}




void loop() {
  bno.GetBNOData();
  double current_yaw = bno.GetYaw();
  Serial.print("Angulo BNO: ");
  Serial.println(current_yaw);
  
    if (Serial1.available()) {
      Serial.println("Datos disponibles en Serial1");
      int bytesRead = Serial1.readBytesUntil('\n', buffer, sizeof(buffer));
      buffer[bytesRead] = '\0';
      Serial.print("Raw Data: ");
      Serial.println(buffer);
      char* token = strtok(buffer, " ");
      ball_distance = atof(token);
      Serial.print("Distancia pelota: ");
      Serial.println(ball_distance);
      token = strtok(NULL, " ");
      ball_angle = atof(token);
      Serial.print("Angulo pelota: ");
      Serial.println(ball_angle);
      token = strtok(NULL, " ");
      goal_distance = atof(token);
      Serial.print("Distancia porteria: ");
      Serial.println(goal_distance);
      token = strtok(NULL, " ");
      goal_angle = atof(token);
      Serial.print("Angulo porteria: ");
      Serial.println(goal_angle);
      opp_goal_distance = atof(token);
      Serial.print("Distancia porteria: ");
      Serial.println(goal_distance);
      token = strtok(NULL, " ");
      opp_goal_angle = atof(token);
      Serial.print("Angulo porteria: ");
      Serial.println(goal_angle);
      open_ball_seen = (ball_distance != 0 || ball_angle != 0);
      if (open_ball_seen){
        Serial.println("Se ha detectado la pelota");
      }
     delay(50);
    } else {
      Serial.println("No se está recibiendo información");
    }


  double error = bno.analize_error(setpoint,current_yaw);
  double speed_w = pid.Calculate(setpoint, error); //Checar si esta bien asi o hay que invertir los valores y aplicar la logica para los diversos
 
  if (speed_w != 0) {
    if (open_ball_seen){
      double error_ball = bno.analize_error(ball_angle, current_yaw);
      double differential_ball = error_ball * 0.001; //Calcular el error diferecial
      ponderated_ball = ball_angle + differential_ball;
      motors.MoveMotorsImu(ponderated_ball, abs(speed_ball), speed_w);
      Serial.println("Moviendo hacia la pelota ");
    }
  }
}


 


 

