#ifndef photo_h
#define photo_h 
#include "constantes.h"
#include <Arduino.h>

class Photo
{
    public:
        Photo();
        void InitializePhoto();
        int ReadPhotoLeft();
        int ReadPhotoRight();
        int ReadPhotoBack();
        int ReadPhotoFront();
        bool PhotoLeft();
        bool PhotoRight();
        bool PhotoBack();
        bool PhotoFront();
        bool EvaluatePhotoState();

    private:
        //Average values Photo left
        uint16_t values_photo_left[analogLeftElements];
        int average_photo_left;
        //Average values Photo right
        uint16_t values_photo_right[analogRightElemts];
        int average_photo_right;
        //Average values Photo back
        uint16_t values_photo_back[analogBackElemts];
        int average_photo_back;
        //Average values Photo front
        uint16_t values_photo_front[analogFrontElemts];
        int average_photo_front;
};

#endif
