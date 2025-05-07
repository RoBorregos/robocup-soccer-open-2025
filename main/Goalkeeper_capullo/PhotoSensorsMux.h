#ifndef PHOTOSENSORSMUX_H
#define PHOTOSENSORSMUX_H

#include <Arduino.h>

// Lados posibles para los sensores
enum Side { FRONT, LEFT, RIGHT, BACK };

class PhotoSensorsMux {
public:
    // Estructura para cada sensor: en qué MUX está y en qué canal
    struct Sensor {
        uint8_t muxIndex;   // Índice del MUX (0-3)
        uint8_t channel;    // Canal del MUX (0-7)
    };

    PhotoSensorsMux(const uint8_t selectPins[3], const uint8_t muxPins[4]);

    void begin();

    void configureSide(Side side, const Sensor* sensors, uint8_t count);
    void setThreshold(Side side, int threshold);

    bool isLineDetected(Side side);
    float getAverage(Side side);       // Valor promedio con smoothing
    float getRawAverage(Side side);    // Lectura directa para calibración

private:
    uint8_t _selectPins[3];     // Pines S0, S1, S2
    uint8_t _muxPins[4];        // Pines de salida de cada MUX (A6, A7, A9, A14)

    int thresholds[4];          // Umbrales por lado

    Sensor* frontSensors;
    uint8_t frontCount;

    Sensor* leftSensors;
    uint8_t leftCount;

    Sensor* rightSensors;
    uint8_t rightCount;

    Sensor* backSensors;
    uint8_t backCount;

    void selectChannel(uint8_t channel);
    int readSensor(uint8_t muxIndex, uint8_t channel);
    float readAverage(const Sensor* sensors, uint8_t size);
};

#endif