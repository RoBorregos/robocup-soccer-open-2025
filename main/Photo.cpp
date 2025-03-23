#include "Photo.h"

Photo::Photo()

Photo::InitializePhoto(){
    for (int i = 0; i < analogLeftElements; i++)
    {
        pinMode(photos_left[i], INPUT);
    }
    for (int i = 0; i < analogRightElements; i++)
    {
        pinMode(photos_right[i], INPUT);
    }
    for (int i = 0; i < analogBackElements; i++)
    {
        pinMode(photos_back[i], INPUT);
    }
    for (int i = 0; i < analogFrontElements; i++)
    {
        pinMode(photos_front[i], INPUT);
    }
}

Photo::ReadPhotoLeft(){
    int sum = 0;
    for (int i = 0; i < analogLeftElements; i++)
    {
        values_photo_left[i] = analogRead(photos_left[i]);
        sum += values_photo_left[i];
    }
}

Photo::ReadPhotoRight(){
    int sum = 0;
    for (int i = 0; i < analogRightElements; i++)
    {
        values_photo_right[i] = analogRead(photos_right[i]);
        sum += values_photo_right[i];
    }
}


Photo::ReadPhotoBack(){
    int sum = 0;
    for (int i = 0; i < analogBackElements; i++)
    {
        values_photo_back[i] = analogRead(photos_back[i]);
        sum += values_photo_back[i];
    }
}

Photo::ReadPhotoFront(){
    int sum = 0;
    for (int i = 0; i < analogFrontElements; i++)
    {
        values_photo_front[i] = analogRead(photos_front[i]);
        sum += values_photo_front[i];
    }
}

bool Photo::PhotoLeft(){
    int left = ReadPhotoLeft();
    return left < lineThreshold_left;
}

bool Photo::PhotoRight(){
    int right = ReadPhotoRight();
    return right < lineThreshold_right;
}

bool Photo::PhotoBack(){
    int back = ReadPhotoBack();
    return back < lineThreshold_back;
}

bool Photo::PhotoFront(){
    int front = ReadPhotoFront();
    return front < lineThreshold_front;
}
