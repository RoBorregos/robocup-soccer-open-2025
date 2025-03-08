
  #include "motors.h"
  #include "constants.h"
  #include "Bno.h"
  #include "PID.h"
  #include "Photo.h"



  float bno_angle = 0;
  unsigned long start_millis;
  unsigned long current_millis;
  unsigned long previous_millis = 0;
  const long interval = 20;
  int setpoint = 90 ;
  int translation_angle = 0;
  int adjust_angle = 0;

  BNO055 my_bno;
  Photo photo;

  PID pid(0.65, 0.01 , 0.0, 200);

  Motors motors(
      MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
      MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
      MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
      MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

  void setup() {
    Serial.begin(115200);
    motors.InitializeMotors();
    my_bno.InitializeBNO();
    photo.InitializeADS();
  }

  void loop() {
    my_bno.GetBNOData();
    double current_yaw = my_bno.GetYaw();
    double error = setpoint - current_yaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    double correction = -pid.Calculate(0, error);
    double speed_w = correction;
    if (speed_w != 0){
    Serial.print("Angulo del BNO: ");
    Serial.println(current_yaw);
    Serial.print("Velocidad corregida: ");
    Serial.println(speed_w);
    motors.MoveMotorsImu(0, 0 , speed_w);
    } 
  }
