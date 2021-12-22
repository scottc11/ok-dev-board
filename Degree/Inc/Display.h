#pragma once

#include "main.h"
#include "IS31FL3739.h"

#define OK_PWM_HIGH 255
#define OK_PWM_MID 80
#define OK_PWM_LOW 20

static const int CHAN_DISPLAY_LED_MAP[4][16]{
    {0, 1, 2, 3, 16, 17, 18, 19, 32, 33, 34, 35, 48, 49, 50, 51},
    {4, 5, 6, 7, 20, 21, 22, 23, 36, 37, 38, 39, 52, 53, 54, 55},
    {8, 9, 10, 11, 24, 25, 26, 27, 40, 41, 42, 43, 56, 57, 58, 59},
    {12, 13, 14, 15, 28, 29, 30, 31, 44, 45, 46, 47, 60, 61, 62, 63}};

#define DISPLAY_SQUARE_LENGTH 12
static const int DISPLAY_SQUARE_LED_MAP[DISPLAY_SQUARE_LENGTH] = {0, 1, 2, 3, 7, 11, 15, 14, 13, 12, 8, 4};

class Display
{
public:
    
    uint8_t state[4][16];
    IS31FL3739 ledMatrix;

    Display(I2C *i2c_ptr) : ledMatrix(i2c_ptr) {}

    void init();
    void clear();
    void setChannelLED(int chan, int index, bool on);
    void setSequenceLEDs(int chan, int length, int diviser, bool on);
    void stepSequenceLED(int chan, int currStep, int prevStep, int length);
    void benderCalibration();

    void drawSquare(int chan);
};