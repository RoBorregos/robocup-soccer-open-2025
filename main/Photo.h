#ifndef photo_h
#define photo_h 
#include "constantes.h"
#include <Arduino.h>

class Photo
{
    public:
        Photo(int frontPins[], int backPins[], int leftPins[], int rightPins[], int size_front, int size_back, int size_left, int size_right);
        void ReadZone(int zone, int* values);
        void ReadAll(int* front_values, int* back_values, int* right_values, int* left_values);
        void printValues(int* front_values, int* back_values, int* left_values, int* right_values);
        bool LineDetected(int zone, int threshold);
        bool PhotoFront();
        bool PhotoBack();
        bool PhotoLeft();
        bool PhotoRight();

    private:
        int* front_pins;
        int* back_pins;
        int* right_pins;
        int* left_pins;
        int front_size, back_size, right_size, left_size; 
};

#endif