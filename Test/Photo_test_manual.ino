#include <Arduino.h>

// Pines de los fototransistores
int frontPins[2]  = {A8, A9};
int rightPins[4]   = {A3, A12, A13, A14};
int leftPins[4]  = {A6, A15, A16, A17};
int backPins[4]   = {A0, A1, A2, A7};

// Función para promediar sensores de un grupo
float getAverage(int* pins, int size) {
  int sum = 0;
  for (int i = 0; i < size; i++) {
    sum += analogRead(pins[i]);
  }
  return sum / float(size);
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Iniciando lectura de fototransistores...");
}

void loop() {
  float frontAvg = getAverage(frontPins, 2); 
  float leftAvg  = getAverage(leftPins, 4);
  float rightAvg = getAverage(rightPins, 4);
  float backAvg  = getAverage(backPins, 4);

  Serial.print("FRONT: "); Serial.println(frontAvg);
  Serial.print(" | LEFT: "); Serial.println(leftAvg);
  Serial.print(" | RIGHT: "); Serial.print(rightAvg);
  Serial.print(" | BACK: "); Serial.println(backAvg);

  delay(500); // medio segundo entre lecturas
}