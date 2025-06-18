#include <Servo.h>  
#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
#include "PhotoSensorsMux.h"
#include <cmath>

#define FILTER_SAMPLES 3

float bno_angle = 0;
unsigned long start_millis;
unsigned long current_millis;
unsigned long previous_millis = 0;
float cathethus = 0;
float setpoint = 0 ;
float adjusted_angle = 0;
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
float cathetus_angle = 0;
float cathetus = 0;
float goal_distance = 0;
float own_angle = 0;
float own_distance = 0;
float differential_ball = 0;
float differential_goal = 0;
float differential_dribbler = 0;
bool open_ball_seen = false;
bool dribbler_ball_seen = false;
bool goal_seen = false;
bool ball_captured = false;
bool own_seen = false;
int time_shoot = 2000;
float angular_tolerance = 10.0; 
String serial1_line = "";
String serial2_line = "";
unsigned long lineDetectedTime = 0;
const unsigned long reverseDuration = 150;
unsigned long correctionTime = 300;
bool isAvoidingLine = false;
enum LineDirection { NONE, FRONTE, LEFTE, RIGHTE, BACKE };
LineDirection lastDirection = NONE;
float dribbler_distance_history[FILTER_SAMPLES] = {0};
float dribbler_angle_history[FILTER_SAMPLES] = {0};
float ball_distance_history[FILTER_SAMPLES] = {0};
float ball_angle_history[FILTER_SAMPLES] = {0};
int filter_index = 0;
unsigned long lastVisionUpdate = 0;
const unsigned long visionInterval = 50;

BNO055 bno;
Servo dribbler;


//PID pid(2, 0.00735 , 30, 500);
PID pid(2, 0.001, 0.1, 300); 


Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);
    
PhotoSensorsMux::Sensor front[8] = {
  {3, 0}, {3, 1}, {3, 3}
};
PhotoSensorsMux::Sensor right[8] = {
  {2, 0}, {2, 3}, {2, 6}, {2, 7}
};

PhotoSensorsMux::Sensor left[8] = {
  {0, 1}
};
PhotoSensorsMux::Sensor back[8] = {
  {1, 0}, {1, 3}, {1, 6}, {1, 7}
};

PhotoSensorsMux sensors(selectPins, muxPins); 

void checkLineSensors() {
  // Check all directions
  bool frontDetected = sensors.isLineDetected(FRONT);
  bool leftDetected = sensors.isLineDetected(LEFT);
  bool rightDetected = sensors.isLineDetected(RIGHT);
  bool backDetected = sensors.isLineDetected(BACK);

  // Priority: Front > Back > Sides (adjust as needed)
  if (frontDetected) {
      lineDetectedTime = millis();
      isAvoidingLine = true;
      motors.SetAllSpeeds(100);
      //motors.MoveBackward();
      Serial.println("AVOIDING FRONT LINE (BACKWARD)");
  } 
  else if (backDetected) {
      lineDetectedTime = millis();
      isAvoidingLine = true;
      motors.SetAllSpeeds(100);
      motors.MoveForward();
      Serial.println("AVOIDING BACK LINE (FORWARD)");
  }
  /*else if (leftDetected) {
      lineDetectedTime = millis();
      isAvoidingLine = true;
      motors.SetAllSpeeds(100);
      //motors.MoveRight();  // Or motors.RotateRight() depending on your lib
      Serial.println("AVOIDING LEFT LINE (RIGHT)");
  }*/
  else if (rightDetected) {
      lineDetectedTime = millis();
      isAvoidingLine = true;
      motors.SetAllSpeeds(100);
      motors.MoveLeft();  // Or motors.RotateLeft()
      Serial.println("AVOIDING RIGHT LINE (LEFT)");
  }
  // Exit condition (checks all possible avoidance cases)
  else if (isAvoidingLine) {
      if (millis() - lineDetectedTime >= correctionTime) {
          isAvoidingLine = false;
          Serial.println("AVOIDANCE ENDED");
      }
  }
}
  
void processSerial1(String line) {
  float dist, ang;
  int parsed = sscanf(line.c_str(), "%f %f", &dist, &ang);
  if (parsed == 2) {
    dribbler_distance = dist;
    //Serial.print("ball_distance 1 ");
    //Serial.println(dribbler_distance);
    dribbler_angle = ang;
    //Serial.print("Angulo 1 ");
   // Serial.println(dribbler_angle);
    dribbler_ball_seen = (dist != 0 && ang != 0);
    ball_captured = (dist <= 20 && ang == 0);
  }
}

void processSerial2(String line) {
  float dist, ang, g_ang, g_dist, o_ang, o_dist;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f", &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);
  if (parsed == 6) {
    ball_distance = dist;
    Serial.print("ball_distance  ");
    Serial.println(ball_distance);
    ball_angle = ang;
    Serial.print("angle 2 ");
    Serial.println(ball_angle);
    goal_distance = g_dist;
    //Serial.print("goal distance ");
    //Serial.println(goal_distance);
    goal_angle = g_ang;
    //Serial.print("goal angle ");
    //Serial.println(goal_angle);
    own_distance = o_dist;
    //Serial.print("own distance ");
    //Serial.println(own_distance);
    own_angle = o_ang;
    //Serial.print("own angle ");
    //Serial.println(own_angle);
    open_ball_seen = !(dist == 0.0f || ang == 0.0f);
    goal_seen = !(g_ang == 0.0f || g_dist == 0.0f);
    own_seen = !(o_ang == 0 || o_dist != 0.0f);
    //own_aligned = (own_angle > 170.f || own_angle < -170.0f);
  }
}

// Procesamiento robusto para Serial1 (dribbler)
void readSerialLines() {
  // Leer desde Serial1
  while (Serial1.available()) {
    char c = Serial1.read();
    //Serial.println(c);
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
    Serial.print(c);
    if (c == '\n') {
      processSerial2(serial2_line);
      serial2_line = "";
      Serial.println("\n");
    } else {
      serial2_line += c;
    }
  }
}



void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial1.begin(115200);
  dribbler.attach(6);
  dribbler.writeMicroseconds(servo_min);
  motors.InitializeMotors();
  bno.InitializeBNO(); 
  //Inicializar Photos
  sensors.begin();
  analogReadResolution(12);
  sensors.configureSide(FRONT, front, 3);
  sensors.configureSide(RIGHT,  right,  4);
  sensors.configureSide(LEFT, left, 1);
  sensors.configureSide(BACK,  back,  4);
  motors.SetAllSpeeds(75);
  //motors.MoveForward();

  delay(500);

}


void loop() { 
  //Comentaste todas las lineas de estados de pelotas menos las del dribbler.
  //Serial.println(sensors.readSensor(3, 7));
  //Serial.println(sensors.getRawAverage(RIGHT));
  //delay(100);
  //dribbler.writeMicroseconds(servo_mid);
  bno.GetBNOData();
  double current_yaw =bno.GetYaw();

  if (millis() - lastVisionUpdate >= visionInterval) {
        readSerialLines();
        lastVisionUpdate = millis();
    }


  double error = bno.analize_error(setpoint, current_yaw);
  double speed_w = pid.Calculate(setpoint, error);
 

  double speed_goal = 90;
  double speed_ball = 90;
  //checkLineSensors();

  //calculateCathetus(ball_distance, own_distance ,ball_angle, own_angle);

if (!isAvoidingLine) {
  if (speed_w != 0){
    if (open_ball_seen && !dribbler_ball_seen){
      double error_ball = ball_angle + current_yaw;
      double differential_ball = error_ball * 0.1; //Calcular el error diferecial
      ponderated_ball = (ball_angle + differential_ball);
      //Serial.print("Angulo ponderado: ");
      //Serial.println(ponderated_ball);
      //setpoint = ponderated_ball;
      //double error = bno.analize_error(setpoint, current_yaw);
      //Serial.print("error: ");
      //Serial.println(error);
      //double speed_w = pid.Calculate(setpoint, error);
      //Serial.print("Correccion: ");
      //Serial.println(speed_w);
      motors.MoveMotorsImu(ponderated_ball, abs(speed_ball), speed_w);
    } 
    } 
  } 
  /*if (goal_distance > 100) {
    motors.StopMotors();
    /*while (goal_distance > 40) {
    motors.SetAllSpeeds(100);
    motors.MoveForward();
    }*/
  }







