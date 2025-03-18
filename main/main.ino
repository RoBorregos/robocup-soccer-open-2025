
#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
#include <cmath>
//Incluir la libreria del servo para el Kicker
//#include "Photo.h"
#include <Pixy2SPI_SS.h>

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


BNO055 my_bno;
Pixy2SPI_SS pixy;
//Photo photo;

PID pid(0.65, 0.01 , 0.6, 200);

Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup() {
  Serial.begin(115200);
  motors.InitializeMotors();
  my_bno.InitializeBNO();
  if (pixy.init()) {
  Serial.println("Éxito");

  // Obtener la resolución de la cámara
  uint16_t width = pixy.frameWidth;
  uint16_t height = pixy.frameHeight;
  Serial.print("Resolución: ");
  Serial.print(width);
  Serial.print("x");
  Serial.println(height);
} else {
  Serial.println("Error");
  while (1); // Detiene el programa si no se inicializa correctamente
}
  //photo.InitializeADS();
}

double analize_error(double setpoint, double current_yaw) {
  double error = setpoint - current_yaw;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return -error; // Checar si aplica el valor negativo aqui
}

double radians_degrees(double radians) {
  return radians * (180.0 / M_PI);
}

void loop() {
  my_bno.GetBNOData();
  double current_yaw = my_bno.GetYaw();

  if (Serial1.available()){
    int bytesRead = Serial1.readBytesUntil('\n', buffer, sizeof(buffer));
    buffer[bytesRead] = '\0';
    char* token = strtok(buffer, " ");
    ball_distance = atof(token);

    token = strtok(NULL, " ");
    ball_angle = atof(token);

    token = strtok(NULL, " ");
    goal_angle = atof(token);

    token = strtok(NULL, " ");
    goal_distance = atof(token);

    open_ball_seen = (ball_distance != 0 || ball_angle != 0);
  }

  int i;
  pixy.ccc.getBlocks();
  int pix_blocks = pixy.ccc.numBlocks;
  if (pixy.ccc.numBlocks) {
    for (i = 0;  i < pixy.ccc.numBlocks; i++) {
      double pix_x = pixy.ccc.blocks[i].m_x;
      double pix_y = pixy.ccc.blocks[i].m_y;
      double dist_x = pix_x - 158;
      double dist_y = pix_y - 208;
      double vector = sqrt(dist_x * dist_x + dist_y * dist_y);
      double distance = 1.2415 * vector - 5.2805;

      double angle_radians = atan2(dist_y, dist_x);
      angle_degrees = radians_degrees(angle_radians);

      angle_degrees += 95;
      if (angle_degrees < 0)
      {
        angle_degrees += 360;
      }
      if (angle_degrees >= 360)
      {
        angle_degrees -= 360;
      }

      Serial.print("ANGLE PIXY: ");
      Serial.println(angle_degrees);

      if (pixy.ccc.blocks[i].m_signature == 1)
      {
        pixy_seen = true;
      }
      else
      {
        pixy_seen = false;
      }
    }
  }

  double error = analize_error(setpoint,current_yaw);
  double speed_w = pid.Calculate(0, error); //Checar si esta bien asi o hay que invertir los valores y aplicar la logica para los diversos casos
  double speed_goal = 155;
  double speed_ball = 200;
  
  if (speed_w != 0) { 

    if (open_ball_seen && !pixy_seen){
      double error_ball =  analize_error(ball_angle, current_yaw);
      double differential_ball = error_ball * 0.001; //Calcular el error diferecial por medio de experimentacion 
      ponderated_ball = ball_angle + differential_ball;
      motors.MoveMotorsImu(ponderated_ball, abs(speed_ball), speed_w);
    
      if (goal_angle != 0) {
      double error_goal = analize_error(goal_angle, current_yaw);
      double differential_goal = error_goal * 0.001; //Calcular el error diferencial por medio de prueba y error
      ponderated_goal = goal_angle + differential_goal;
      motors.MoveMotorsImu(ponderated_goal, abs(speed_goal), speed_w);

      }
    }

    if (!open_ball_seen && pixy_seen) {
      if (goal_angle != 0 ) {
        double error_goal = analize_error(goal_angle, current_yaw);
        double differential_goal = error_goal * 0.001; //Calcular el error diferencial por medio de prueba y error
        ponderated_goal = goal_angle + differential_goal;
        if (goal_distance > 20){ //Checar si se puede medir la distancia con la OpenMV
          motors.StopMotors();
          //Disparar pelota con Kicker
        } 
      }
    }

//Case of nothing detected
    if (!open_ball_seen || !pixy_seen) {
      double last_error = analize_error(last_angle, current_yaw); 
      motors.MoveMotorsImu(last_angle, abs(speed_ball), speed_w);
    }
}
}

/*
Distancias de la Pixy. 318 x 208
*/
  


  //

  //Ajusten en base a distancia y a angulos de pelota/porteria

  //

  
  