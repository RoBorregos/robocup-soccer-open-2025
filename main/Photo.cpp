#include "Photo.h"

Photo::Photo(int frontPins[], int backPins[], int leftPins[], int rightPins[], int size_front, int size_back, int size_left, int size_right){
    front_pins = frontPins;
    back_pins = backPins;
    right_pins = rightPins;
    left_pins = leftPins;
    front_size = size_front;
    back_size = size_back;
    right_size = size_right;
    left_size = size_left;
}



void Photo::ReadZone(int zone, int* values){
    int* pins;
    int size;

    switch(zone){
        case front_array:
            pins = front_pins;
            size = front_size;
            break;
        case back_array:
            pins = back_pins;
            size = back_size;
            break;
        case right_array:
            pins = right_pins;
            size = right_size;
            break;
        case left_array:
            pins = left_pins;
            size = left_size;
            break;
        default:
            return;
    }

    for (int i = 0; i < size; i++) {
        values[i] = analogRead(pins[i]);
    }
}

void Photo::ReadAll(int* front_values, int* back_values, int* right_values, int* left_values) {
    ReadZone(front_array, front_values);
    ReadZone(back_array, back_values);
    ReadZone(right_array, right_values);
    ReadZone(left_array, left_values);
}

void Photo::printValues(int* front_values, int* back_values, int* left_values, int* right_values) {
    Serial.print("Front: ");
    for (int i = 0; i < front_size; i++) {
        Serial.print(front_values[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("Back: ");
    for (int i = 0; i < back_size; i++) {
        Serial.print(back_values[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("Right: ");
    for (int i = 0; i < right_size; i++) {
        Serial.print(right_values[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("Left: ");
    for (int i = 0; i < left_size; i++) {
        Serial.print(left_values[i]);
        Serial.print(" ");
    }
    Serial.println();
}

bool Photo::LineDetected(int zone){
    int* pins;
    int size;
    int threshold;

    switch(zone){
        case front_array:
            pins = front_pins;
            size = front_size;
            threshold = lineThreshold_front;
            break;
        case back_array:
            pins = back_pins;
            size = back_size;
            threshold = lineThreshold_back;
            break;
        case right_array:
            pins = right_pins;
            size = right_size;
            threshold = lineThreshold_right;
            break;
        case left_array:
            pins = left_pins;
            size = left_size;
            threshold = lineThreshold_left;
            break;
        default:
            return false;
    }
    for (int i = 0; i < size; i++) {
        if (analogRead(pins[i]) > threshold) {
            return true;
        }
    }
    return false;
}

bool Photo::PhotoFront() {
    return LineDetected(front_array);
}

bool Photo::PhotoBack() {
    return LineDetected(back_array);
}
bool Photo::PhotoLeft() {
    return LineDetected(left_array);
}

bool Photo::PhotoRight() {
    return LineDetected(right_array);
}
