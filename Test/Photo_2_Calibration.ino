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
  sensors.begin();

  sensors.configureSide(FRONT, front, 2);
  sensors.configureSide(LEFT,  left,  7);
  sensors.configureSide(RIGHT, right, 8);
  sensors.configureSide(BACK,  back,  8);
}

void loop() {
  Serial.print("FRONT: "); Serial.println(sensors.getRawAverage(FRONT));
  Serial.print("LEFT: ");  Serial.println(sensors.getRawAverage(LEFT)); 
  Serial.print("RIGHT: "); Serial.println(sensors.getRawAverage(RIGHT)); 
  Serial.print("BACK: ");  Serial.println(sensors.getRawAverage(BACK));




  Serial.println("-----");
  delay(500);  // Para que no sature el monitor serial  
}
