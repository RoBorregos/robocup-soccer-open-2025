#ifndef PHOTO_H
#define PHOTO_H

#include <Arduino.h>

enum Side {
  FRONT,
  LEFT,
  RIGHT,
  BACK
};

class PhotoSensors {
  private:
    uint8_t frontPins[2];
    uint8_t leftPins[4];
    uint8_t rightPins[4];
    uint8_t backPins[4];
    int thresholds[4]; // One for each side

    float readAverage(uint8_t* pins, uint8_t size);

  public:
    PhotoSensors(uint8_t f[2], uint8_t l[4], uint8_t r[4], uint8_t b[4]);

    void setThreshold(Side side, int t);
    float getAverage(Side side);
    bool isLineDetected(Side side);
};

#endif

