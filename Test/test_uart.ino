#include <PWMServo.h>  
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
bool pixy_seen = false;
const int BUFFER_SIZE = 50;
char buffer[BUFFER_SIZE];
const int servo_min = 1000;
const int servo_mid = 1500;
const int servo_max = 2000;
int time_shoot = 2000;


BNO055 bno;
PWMServo servo;
//Photo photo;


PID pid(0.65, 0.01 , 0.6, 200);


Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}


void loop() { 
    if (Serial1.available()) {
      Serial.println("Datos disponibles en Serial1");
  
      int bytesRead = Serial1.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
      buffer[bytesRead] = '\0'; // Agregar terminador de cadena
  
      Serial.print("Raw Data: ");
      Serial.println(buffer);
    } else {
      Serial.println("No se está recibiendo información");
    }
  
    delay(500);
  }
  