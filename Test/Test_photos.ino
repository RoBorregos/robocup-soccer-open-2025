#include <Servo.h>  
#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
#include "PhotoSensorsMux.h"
#include <cmath>
//Incluir la libreria del servo para el Kicker




float bno_angle = 0;
unsigned long start_millis;
unsigned long current_millis;
unsigned long previous_millis = 0;
float cathethus = 0;
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
float own_angle = 0;
float own_distance = 0;
float distance_pixels = 0;
float differential_ball = 0;
float differential_goal = 0;
float differential_dribbler = 0;
bool open_ball_seen = false;
bool dribbler_ball_seen = false;
bool goal_seen = false;
bool ball_captured = false;
bool own_seen = false;
const int BUFFER_SIZE = 100;
char buffer1[BUFFER_SIZE];
char buffer2[BUFFER_SIZE];
const int servo_min = 1000;
const int servo_mid = 1700;
const int servo_max = 2000;
int time_shoot = 2000;
float angular_tolerance = 10.0;
String serial1_line = "";
String serial2_line = "";


unsigned long lineDetectedTime = 0;
const unsigned long reverseDuration = 150;
bool isAvoidingLine = false;
enum LineDirection {NONE, FRONTE, LEFTE, RIGHTE, BACKE};
LineDirection lastDirection = NONE;
const uint8_t selectPins[3] = {16, 15, 14};  // S0, S1, S2
const uint8_t muxPins[4] = {A6, A7, A9, A14};


BNO055 bno;
Servo dribbler;


PhotoSensorsMux sensors(selectPins, muxPins);


// Ejemplo de configuración (ajusta a tu conexión real)


PID pid(6, 0.01 , 0.6, 600); //0.6, 0.01, 0.6, 200
PID pid2(3, 0.01 , 0.6, 500);




PhotoSensorsMux::Sensor front[8] = {
    {3, 1}, {3, 5}
};
PhotoSensorsMux::Sensor left[8] = {
    {0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 6}, {0, 7}
};
PhotoSensorsMux::Sensor right[8] = {
    {2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}, {2, 5}, {2, 6}, {2, 7}
};
PhotoSensorsMux::Sensor back[8] = {
    {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}
};




Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);




void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial1.begin(115200);
  //dribbler.attach(6);
  //dribbler.writeMicroseconds(servo_min);
  motors.InitializeMotors();
  bno.InitializeBNO();
  sensors.begin();


  sensors.configureSide(FRONT, front, 2);
  sensors.configureSide(LEFT,  left,  4);
  sensors.configureSide(RIGHT, right, 4);
  sensors.configureSide(BACK,  back,  4);
  delay(1000);
}




void loop() {
  //Serial.print("FRONT: "); Serial.println(sensors.getRawAverage(FRONT));
  //Serial.print("LEFT: ");  Serial.println(sensors.getRawAverage(LEFT));
  //Serial.print("RIGHT: "); Serial.println(sensors.getRawAverage(RIGHT));
  //Serial.print("BACK: ");  Serial.println(sensors.getRawAverage(BACK));
  motors.SetAllSpeeds(80);
  motors.MoveForward();
 
  checkLineSensors();
  //dribbler.writeMicroseconds(servo_mid);
  bno.GetBNOData();
  double current_yaw = bno.GetYaw();
  readSerialLines();
 
  cathethus = calculateCathetus(ball_distance, own_distance, ball_angle, own_angle);


  double error = bno.analize_error(setpoint,current_yaw);
  double speed_w = pid.Calculate(setpoint, error);
  double speed_d = pid2.Calculate(setpoint, error); //Checar si esta bien asi o hay que invertir los valores y aplicar la logica para los diversos casos
  double speed_goal = 150;
  double speed_ball = 150;
 
if (cathethus < 90){
  if (open_ball_seen && !dribbler_ball_seen){
    double error_ball = ball_angle + current_yaw;
    double differential_ball = error_ball * 0.001; //Calcular el error diferecial
    ponderated_ball = (ball_angle + differential_ball);
    setpoint = ponderated_ball;
    error = bno.analize_error(setpoint,current_yaw);
    speed_w = pid.Calculate(setpoint, error);  
    motors.MoveMotorsImu(ponderated_ball, abs(speed_ball), speed_w);
  } else if (dribbler_ball_seen){
    double error_dribbler = dribbler_angle + current_yaw;
    double differential_dribbler = error_dribbler * 0.001; //Calcular el error diferecial
    ponderated_dribbler = -(dribbler_angle + differential_dribbler);
    setpoint = ponderated_dribbler;
    error = bno.analize_error(setpoint,current_yaw);
    speed_d = pid2.Calculate(setpoint, error);
    motors.MoveMotorsImu(ponderated_dribbler, abs(speed_ball), speed_d);
    if (ball_captured){
      if (goal_seen && goal_angle != 0){
        double error_goal= goal_angle + current_yaw;
        double differential_goal = error_goal * 0.001; //Calcular el error diferecial
        ponderated_goal = (goal_angle + differential_goal);
        setpoint = ponderated_goal;
        error = bno.analize_error(setpoint,current_yaw);
        speed_d = pid2.Calculate(setpoint, error);
        motors.MoveMotorsImu(ponderated_goal, abs(speed_goal), speed_w);
      } else if (goal_angle == 0 && goal_seen){
        motors.StopMotors();
        digitalWrite(KICKER_PIN, HIGH);
        delay(20);
        digitalWrite(KICKER_PIN, LOW);
      }
    }
  }  
      } else {
        if (own_seen){
          float adjusted_angle = own_angle + 180.0f;


          if (adjusted_angle >= 360.0f) {
            adjusted_angle -= 360.0f; // Asegurarse de que el ángulo esté en el rango [0, 360)
          }
          setpoint = adjusted_angle;
          double error_adjustment = bno.analize_error(setpoint, current_yaw);
          speed_w = pid.Calculate(setpoint, error_adjustment);
          double error_own= own_angle + current_yaw;


          if (fabs(error_own) > angular_tolerance) {
            motors.MoveMotorsImu(0, 0, speed_w);  // Girar robor a que se alinee con el frente de la cancha
          } else{
          double differential_own = error_own * 0.001; //Calcular el error diferecial
          ponderated_goal = (own_angle + differential_own);
          setpoint = goal_angle;
          error = bno.analize_error(setpoint,current_yaw);
          speed_w = pid.Calculate(setpoint, error);
          motors.MoveMotorsImu(ponderated_goal, abs(speed_goal), speed_w);
          }
        }
      }
}


void checkLineSensors() {
  if (!isAvoidingLine) {
    if (sensors.isLineDetected(FRONT)) {
      Serial.println("Line detected in front!");
      motors.SetAllSpeeds(120);
      motors.MoveBackward();
      isAvoidingLine = true;
      lastDirection = FRONTE;
      lineDetectedTime = millis();
      } else if (sensors.isLineDetected(LEFT)) {
        Serial.println("Line detected on left!");
        motors.SetAllSpeeds(120);
        motors.MoveRight();
        isAvoidingLine = true;
        lastDirection = LEFTE;
        lineDetectedTime = millis();
        } else if (sensors.isLineDetected(RIGHT)) {
          Serial.println("Line detected on right!");
          motors.SetAllSpeeds(120);
          motors.MoveLeft();
          isAvoidingLine = true;
          lastDirection = RIGHTE;
          lineDetectedTime = millis();
          } else if (sensors.isLineDetected(BACK)) {
            Serial.println("Line detected on back!");
            motors.SetAllSpeeds(120);
            motors.MoveBackward();
            isAvoidingLine = true;
            lastDirection = BACKE;
            lineDetectedTime = millis();
          }
  } else {
  // Check if enough time has passed to stop avoiding
    if (millis() - lineDetectedTime >= reverseDuration) {
    isAvoidingLine = false;
    lastDirection = NONE;
    }
  }
}
     








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
    //Serial.print("ball_distance 1 ");
    //Serial.println(dribbler_distance);
    dribbler_angle = ang;
    //Serial.print("Angulo 1 ");
    //Serial.println(dribbler_angle);
    dribbler_ball_seen = !(dist == 0.0f || ang == 0.0f); // Mejor expresado
    ball_captured = (dribbler_distance <= 20.0f && dribbler_angle == 0.0f && dribbler_distance > 0); // Actualiza SIEMPRE
    if (ball_captured) {
      Serial.println("Pelota capturada");
    }
  } else {
    dribbler_ball_seen = false;
    ball_captured = false;
  }
}


void processSerial2(String line) {
  float dist, ang, g_ang, g_dist, o_ang, o_dist;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f", &dist, &ang, &g_ang, &g_dist, &o_ang, &o_dist);
  if (parsed == 6) {
    ball_distance = dist;
    //Serial.print("ball_distance 2 ");
    //Serial.println(ball_distance);
    ball_angle = ang;
    //Serial.print("ball_distance 2 ");
    //Serial.println(ball_angle);
    goal_distance = g_dist;
    //Serial.print("goal distance ");
    //Serial.println(goal_distance);
    goal_angle = g_ang;
    //Serial.print("goal angle ");
    //Serial.println(goal_angle);
    own_distance = o_dist;
    Serial.print("own distance ");
    Serial.println(own_distance);
    own_angle = o_ang;
    Serial.print("own angle ");
    Serial.println(own_angle);
    open_ball_seen = !(dist == 0.0f || ang == 0.0f);
    goal_seen = !(g_ang == 0.0f || g_dist == 0.0f);
    own_seen = !(o_ang == 0 || o_dist != 0.0f);
  }
}


float calculateCathetus(float a, float b, float angle, float angle2) {
  float adjusted_angle = angle2 - angle; // Ajustar el ángulo
  if (adjusted_angle < 0) {
    adjusted_angle += 360; // Asegurarse de que el ángulo esté en el rango [0, 360)
  }
  if (adjusted_angle > 180) {
    adjusted_angle -= 360; // Asegurarse de que el ángulo esté en el rango [-180, 180)
  }
  float angle_rad = angle * M_PI / 180.0; // Convertir a radianes
  float cathethus = sqrtf(a*a + b*b - 2 * a * b * cos(angle_rad));
  return cathethus;
}
