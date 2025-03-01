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

BNO055 my_bno;

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
  motors.SetAllSpeeds(50);
  my_bno.GetBNOData();
  bno_angle = my_bno.GetYaw();
  Serial.println(bno_angle);

  Serial.println("Mover hacia adelante");
  motors.MoveForward();
  delay(2000);

  Serial.println("Mover hacia atras");
  motors.MoveBackward();
  delay(2000);

  Serial.println("Mover hacia la izquierda");
  motors.MoveLeft();
  delay(2000);

  Serial.println("Mover hacia la derecha");
  motors.MoveRight();
  delay(2000);

}


/*
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}
*/