#include "photo.h"

PhotoSensors::PhotoSensors(uint8_t f[2], uint8_t l[4], uint8_t r[4], uint8_t b[4]) {
  memcpy(frontPins, f, 2);
  memcpy(leftPins, l, 4);
  memcpy(rightPins, r, 4);
  memcpy(backPins, b, 4);

  // Default thresholds for each side
  thresholds[FRONT] = 500;
  thresholds[LEFT]  = 500;
  thresholds[RIGHT] = 500;
  thresholds[BACK]  = 500;
}

void PhotoSensors::setThreshold(Side side, int t) {
  thresholds[side] = t;
}

float PhotoSensors::readAverage(uint8_t* pins, uint8_t size) {
  int sum = 0;
  for (uint8_t i = 0; i < size; i++) {
    sum += analogRead(pins[i]);
  }
  return sum / float(size);
}

float PhotoSensors::getAverage(Side side) {
  switch (side) {
    case FRONT: return readAverage(frontPins, 2);
    case LEFT:  return readAverage(leftPins, 4);
    case RIGHT: return readAverage(rightPins, 4);
    case BACK:  return readAverage(backPins, 4);
    default:    return 0;
  }
}

bool PhotoSensors::isLineDetected(Side side) {
  return getAverage(side) > thresholds[side];
}
