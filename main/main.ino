#include "motors.h"
#include "constants.h"
#include "Bno.h"
#include "PID.h"
#include "Photo.h"



float bno_angle = 0;
unsigned long start_millis;
unsigned long current_millis;
int setpoint = 90 ;
int translation_angle = 0;
int adjust_angle = 0;

BNO055 my_bno;
Photo photo;

PID pid(0.5, 0.1 , 0.8, 400);

Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup() {
  Serial.begin(115200);
  motors.InitializeMotors();
  my_bno.InitializeBNO();
  start_millis = millis();
  photo.InitializeADS();
}

void loop() {

  my_bno.GetBNOData();
  double current_yaw = my_bno.GetYaw();
  Serial.print("Angulo del BNO: ");
  Serial.println(current_yaw);
  double correction = pid.Calculate(setpoint, current_yaw);
  double speed_w = correction;
  speed_w = constrain(speed_w, -100, 100);
  Serial.print("Velocidad corregida: ");
  Serial.println(speed_w);
  motors.MoveMotorsImu(setpoint, 100 , speed_w);
  delay(50);
}
