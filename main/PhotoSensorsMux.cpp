#include "PhotoSensorsMux.h"
#include <Arduino.h>

PhotoSensorsMux::PhotoSensorsMux(const uint8_t selectPins[3], const uint8_t muxPins[4]) {
    memcpy(_selectPins, selectPins, 3);
    memcpy(_muxPins, muxPins, 4);

    thresholds[FRONT] = 2100;
    thresholds[LEFT] = 1500;
    thresholds[RIGHT] = 2200;
    thresholds[BACK] = 3045;

    frontSensors = nullptr;
    leftSensors = nullptr;
    rightSensors = nullptr;
    backSensors = nullptr;

    frontCount = leftCount = rightCount = backCount = 0;
}

void PhotoSensorsMux::begin() {
    for (int i = 0; i < 3; i++) {
        pinMode(_selectPins[i], OUTPUT);
    }
}

void PhotoSensorsMux::configureSide(Side side, const Sensor* sensors, uint8_t count) {
    Sensor* ptr = new Sensor[count];
    memcpy(ptr, sensors, count * sizeof(Sensor));

    switch (side) {
        case FRONT: frontSensors = ptr; frontCount = count; break;
        case LEFT:  leftSensors = ptr;  leftCount = count;  break;
        case RIGHT: rightSensors = ptr; rightCount = count; break;
        case BACK:  backSensors = ptr;  backCount = count;  break;
    }
}

void PhotoSensorsMux::setThreshold(Side side, int threshold) {
    thresholds[side] = threshold;
}

void PhotoSensorsMux::selectChannel(uint8_t channel) {
    digitalWrite(_selectPins[0], channel & 0x01);
    digitalWrite(_selectPins[1], (channel >> 1) & 0x01);
    digitalWrite(_selectPins[2], (channel >> 2) & 0x01);
    delayMicroseconds(5); // Estabilización
}

int PhotoSensorsMux::readSensor(uint8_t muxIndex, uint8_t channel) {
    selectChannel(channel);
    return analogRead(_muxPins[muxIndex]);
}

float PhotoSensorsMux::readAverage(const Sensor* sensors, uint8_t size) {
    int sum = 0;
    for (uint8_t i = 0; i < size; i++) {
        sum += readSensor(sensors[i].muxIndex, sensors[i].channel);
    }
    return sum / float(size);
}

float PhotoSensorsMux::getAverage(Side side) {
    switch (side) {
        case FRONT: return readAverage(frontSensors, frontCount);
        case LEFT:  return readAverage(leftSensors, leftCount);
        case RIGHT: return readAverage(rightSensors, rightCount);
        case BACK:  return readAverage(backSensors, backCount);
        default:    return 0;
    }
}

float PhotoSensorsMux::getRawAverage(Side side) {
    return getAverage(side);  // Podrías hacer un filtro más crudo si lo deseas
}

bool PhotoSensorsMux::isLineDetected(Side side) {
    return getAverage(side) > thresholds[side];
}
