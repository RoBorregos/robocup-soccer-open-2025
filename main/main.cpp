#include <Servo.h>  
#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
#include "PhotoSensorsMux.h"
#include <cmath>

#define FILTER_SAMPLES 3

const uint8_t selectPins[3] = {16, 15, 14};  // S0, S1, S2
const uint8_t muxPins[4] = {A6, A7, A9, A14}; // Pines de salida MUX

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
PID pid(0.47, 0.001, 0.01, 500); 
//PID pid(0.1, 0.000, 0.0, 200); //0.6, 0.01, 0.6, 200
PID pid2(0.001, 0.001 , 1.1, 500);


Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);
    
PhotoSensorsMux::Sensor front[8] = {
  {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}
};
PhotoSensorsMux::Sensor right[8] = {
  {2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 5}, {2, 6}, {2, 7}
};

PhotoSensorsMux::Sensor left[8] = {
  {0, 1}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7}
};
PhotoSensorsMux::Sensor back[8] = {
  {3, 2}, {3, 4}, {3, 5}, {3, 6}, {3, 7}
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
      motors.MoveBackward();
      Serial.println("AVOIDING FRONT LINE (BACKWARD)");
  } 
  else if (backDetected) {
      lineDetectedTime = millis();
      isAvoidingLine = true;
      motors.SetAllSpeeds(100);
      motors.MoveForward();
      Serial.println("AVOIDING BACK LINE (FORWARD)");
  }
  else if (leftDetected) {
      lineDetectedTime = millis();
      isAvoidingLine = true;
      motors.SetAllSpeeds(100);
      motors.MoveRight();  // Or motors.RotateRight() depending on your lib
      Serial.println("AVOIDING LEFT LINE (RIGHT)");
  }
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




float calculateCathetus(float a, float b, float angle, float angle2) {
  //Serial.print("a :");
  //Serial.println(a);
  //Serial.print("b :");
  //Serial.println(b);
  float adjusted_angle = abs(angle2 - angle);
  //Serial.print("Angulo ajustado: "); // Ajustar el ángulo
  //Serial.println(adjusted_angle);
  if (adjusted_angle > 180) {
    cathetus_angle = 360 - adjusted_angle; //Normalizar el angulo
  } else {
    cathetus_angle = adjusted_angle;
  }
  //Serial.print("Angulo cateto: ");
  //Serial.println(cathetus_angle);
  float angle_rad = cathetus_angle * M_PI / 180.0;
  //Serial.print("Angulo radianes: ");
  //Serial.println(angle_rad); // Convertir a radianes
  double cathetus = sqrt(a*a + b*b - 2 * a * b * cos(angle_rad));
  //Serial.print("Cateto: ");
  //Serial.println(cathetus);

  return cathetus;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  motors.InitializeMotors();
  sensors.begin();

  sensors.configureSide(FRONT, front, 7);
  sensors.configureSide(LEFT,  left,  7);
  //sensors.configureSide(RIGHT, right, 4);
  sensors.configureSide(BACK,  back,  5);
}

void loop() {
    motors.SetAllSpeeds(110);
    motors.MoveForward();
}







