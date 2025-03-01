/*
#include <PIDLoop.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <Pixy2I2C.h>
#include <Pixy2Line.h>
#include <Pixy2SPI_SS.h>
#include <Pixy2UART.h>
#include <Pixy2Video.h>
#include <TPixy2.h>
#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
*/

#include "motors.h"
#include "constants.h"
#include "Bno.h"
#include "PID.h"


float bno_angle = 0;
unsigned long start_millis;
unsigned long current_millis;
int setpoint = 90 ;
int translation_angle = 0;
int adjust_angle = 0;

BNO055 my_bno;

PID pid(1, 0.4 , 1.5, 400);

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
}

void loop() {

  my_bno.GetBNOData();
  double current_yaw = my_bno.GetYaw();
  Serial.print("Angulo del BNO: ");
  Serial.println(current_yaw);
  double correction = pid.Calculate(setpoint, current_yaw);
  double speed_w = correction;
  Serial.print("Velocidad corregida: ");
  Serial.println(speed_w);
  motors.MoveMotorsImu(setpoint, 100 , speed_w);

}
