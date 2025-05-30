#include "PhotoSensorsMux.h"

unsigned long lineDetectedTime = 0;
const unsigned long correctionTime = 250;
bool isAvoidingLine = false;
const uint8_t selectPins[3] = {16, 15, 14};  // S0, S1, S2
const uint8_t muxPins[4] = {A6, A7, A9, A14}; // Pines de salida MUX

PhotoSensorsMux sensors(selectPins, muxPins);

// Ejemplo de configuración (ajusta a tu conexión real)
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

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  motors.InitializeMotors();
  sensors.begin();

  sensors.configureSide(FRONT, front, 2);
  sensors.configureSide(LEFT,  left,  4);
  sensors.configureSide(RIGHT, right, 4);
  sensors.configureSide(BACK,  back,  4);
}

void loop() {
  Serial.print("FRONT: "); Serial.println(sensors.getRawAverage(FRONT));
  Serial.print("LEFT: ");  Serial.println(sensors.getRawAverage(LEFT));
  Serial.print("RIGHT: "); Serial.println(sensors.getRawAverage(RIGHT));
  Serial.print("BACK: ");  Serial.println(sensors.getRawAverage(BACK));




  Serial.println("-----");
  delay(500);  // Para que no sature el monitor serial


  checkLineSensors();
  if (!isAvoidingLine) {
  motors.SetAllSpeeds(110);
  motors.MoveForward();
  }
}

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
/*else if (leftDetected) {
    lineDetectedTime = millis();
    isAvoidingLine = true;
    motors.SetAllSpeeds(100);
    motors.MoveRight();  // Or motors.RotateRight() depending on your lib
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