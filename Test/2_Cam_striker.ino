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
float distance_pixels = 0;
float differential_ball = 0;
float differential_goal = 0;
float differential_dribbler = 0;
bool open_ball_seen = false;
bool dribbler_ball_seen = false;
bool goal_seen = false;
bool ball_captured = false;
const int BUFFER_SIZE = 100;
char buffer1[BUFFER_SIZE];
char buffer2[BUFFER_SIZE];
const int servo_min = 1000;
const int servo_mid = 1500;
const int servo_max = 2000;
int time_shoot = 2000;
String serial1_line = "";
String serial2_line = "";
uint8_t front[2] = {A8, A9};
uint8_t right[4] = {A3, A12, A13, A14};
uint8_t left[4] = {A6, A15, A16, A17};



BNO055 bno;
Servo dribbler;
//Photo photo;


PID pid(6, 0.01 , 0.6, 500); //0.6, 0.01, 0.6, 200
PID pid2(5, 0.01 , 0.6, 500);


Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);
    
PhotoSensors sensors(front, left, right);


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial1.begin(115200);
  dribbler.attach(6);
  dribbler.writeMicroseconds(servo_min);
  motors.InitializeMotors();
  bno.InitializeBNO(); 
 
  sensors.setThreshold(FRONT, 600);
  sensors.setThreshold(LEFT,  580);
  sensors.setThreshold(RIGHT, 590);
  //sensors.setThreshold(BACK,  610);

  delay(1000);

}


void loop() { 
  dribbler.writeMicroseconds(servo_mid);
  bno.GetBNOData();
  double current_yaw = bno.GetYaw();
  readSerialLines();

  double error = bno.analize_error(setpoint,current_yaw);
  double speed_w = pid.Calculate(setpoint, error);
  double speed_d = pid2.Calculate(setpoint, error); //Checar si esta bien asi o hay que invertir los valores y aplicar la logica para los diversos casos
  double speed_goal = 200;
  double speed_ball = 150;
 

  if (open_ball_seen && !dribbler_ball_seen){
    double error_ball = ball_angle + current_yaw;
    double differential_ball = error_ball * 0.001; //Calcular el error diferecial
    ponderated_ball = (ball_angle + differential_ball);
    setpoint = ponderated_ball;
    motors.MoveMotorsImu(ponderated_ball, abs(speed_ball), speed_w);
  } else if (dribbler_ball_seen){
    double error_dribbler = dribbler_angle + current_yaw;
    double differential_dribbler = error_dribbler * 0.001; //Calcular el error diferecial
    ponderated_dribbler = -(dribbler_angle + differential_dribbler);
    setpoint = ponderated_dribbler;
    motors.MoveMotorsImu(ponderated_dribbler, abs(speed_ball), speed_d);
    if (ball_captured){
      if (goal_seen && goal_angle != 0){
        double error_goal= goal_angle + current_yaw;
        double differential_goal = error_goal * 0.001; //Calcular el error diferecial
        ponderated_goal = (goal_angle + differential_goal);
        setpoint = ponderated_goal;
        motors.MoveMotorsImu(ponderated_goal, abs(speed_ball), speed_w);
      } else if (goal_angle == 0 && goal_seen){
        motors.StopMotors();
        //digitalWrite(KICKER_PIN, HIGH);
        //delay(20);
        //digitalWrite(KICKER_PIN, LOW);
      }
    }
} else {
      motors.MoveMotorsImu(0,0,speed_w);
      if (speed_w == 0) {
        //motors.SetAllSpeeds(80);
        //motors.MoveBackward();
        motors.SetAllSpeeds(0);
        motors.StopMotors();
      }
    }
}

if (sensors.isLineDetected(FRONT) == true) {
  Serial.println("Line detected in front!");
  motors.SetAllSpeeds(120);
  motors.MoveBackward();
  delay(150);
} else if (sensors.isLineDetected(LEFT) == true) {
    Serial.println("Line detected on left!");
    motors.SetAllSpeeds(120);
    motors.MoveRight();
    delay(150);
  } else if (sensors.isLineDetected(RIGHT) == true) {
    Serial.println("Line detected on right!");
    motors.SetAllSpeeds(120);
    motors.MoveLeft();
    delay(150);

void readSerialLines() {
  // Leer desde Serial1
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      processSerial1(serial1_line);
      serial1_line = "";
    } else {
      serial1_line += c;
    }
  }

  // Leer desde Serial2
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      processSerial2(serial2_line);
      serial2_line = "";
    } else {
      serial2_line += c;
    }
  }
}

void processSerial1(String line) {
  float dist, ang;
  int parsed = sscanf(line.c_str(), "%f %f", &dist, &ang);
  if (parsed == 2) {
    dribbler_distance = dist;
    Serial.print("ball_distance 1 ");
    Serial.println(dribbler_distance);
    dribbler_angle = ang;
    Serial.print("Angulo 1 ");
    Serial.println(dribbler_angle);
   dribbler_ball_seen = !(dist == 0.0f || ang == 0.0f); // Mejor expresado
    ball_captured = (dribbler_distance <= 20.0f && dribbler_angle == 0.0f && dribbler_distance > 0); // Actualiza SIEMPRE
  } else {
    dribbler_ball_seen = false;
    ball_captured = false;
  }
}

void processSerial2(String line) {
  float dist, ang, g_ang, g_dist;
  int parsed = sscanf(line.c_str(), "%f %f %f %f", &dist, &ang, &g_ang, &g_dist);
  if (parsed == 4) {
    ball_distance = dist;
    Serial.print("ball_distance 2 ");
    Serial.println(ball_distance);
    ball_angle = ang;
    Serial.print("ball_distance 2 ");
    Serial.println(ball_angle);
    goal_angle = g_ang;
    goal_distance = g_dist;
    open_ball_seen = (dist != 0 && ang != 0);
    goal_seen = (g_ang != 0 && g_dist != 0);
  }
}


//te amo emil