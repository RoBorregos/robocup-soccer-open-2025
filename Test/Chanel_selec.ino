// Definición de pines de selección y pin de salida del MUX


#include "PhotoSensorsMux.h"


const uint8_t selectPins[3] = {16, 15,14};  // S0, S1, S2
const uint8_t muxPin = A6;  // Pin donde se conecta la salida del MUX (por ejemplo, A0)


void setup() {
  // Inicializar los pines de selección como salidas
  for (int i = 0; i < 3; i++) {
    pinMode(selectPins[i], OUTPUT);
  }
 
  // Inicializar el pin de lectura
  pinMode(muxPin, INPUT);
 
  Serial.begin(9600);
}


void loop() {
  // Leer todos los canales (del 0 al 7)
  for (int i = 0; i < 8; i++) {
    selectChannel(i);  // Selecciona el canal i
    int sensorValue = analogRead(muxPin);  // Lee el valor del canal seleccionado
    Serial.print("Canal ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensorValue);  // Muestra el valor leído
    delay(500);  // Espera un poco antes de leer el siguiente canal
  }
}


// Función para seleccionar el canal en el MUX
void selectChannel(int channel) {
  digitalWrite(selectPins[0], channel & 0x01);         // S0
  digitalWrite(selectPins[1], (channel >> 1) & 0x01);   // S1
  digitalWrite(selectPins[2], (channel >> 2) & 0x01);   // S2
  delay(5);  // Estabilización de la señal
}
