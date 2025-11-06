#pragma once

#include <cmath>
#include "main.h"
#include "NoteData.h"

// Forward declaration to avoid circular dependency with TouchChannel
namespace DEGREE { class TouchChannel; }

#define NUM_CAPTURE_READINGS 8
const int NUM_POINTS_INTERPOLATION = 40;

struct InterpData
{
    float frequency;   // vco frequency
    uint16_t dacValue; // the corrosponding DAC value for the frequency
};

class Calibrator
{
    public:
        Calibrator() {
            this->channel = NULL;
        };

        int calibrationIndex = 0;
        DEGREE::TouchChannel *channel;
        InterpData interpolationData[NUM_POINTS_INTERPOLATION];
        uint32_t CAPTURE_READINGS[NUM_CAPTURE_READINGS];

        void initVCOCalibration(DEGREE::TouchChannel *targetChannel);
        void handleInputCapture();
        void interpolateTuningTable();
        void captureCallback();
        
        // EEPROM storage
        void storeCalibrationToEEPROM(DEGREE::TouchChannel *targetChannel);
        void loadVCOCalDataFromEEPROM(DEGREE::TouchChannel *targetChannel);
        void clearCalibrationFromEEPROM(DEGREE::TouchChannel *targetChannel);

    private:
};

uint16_t ok_voltage_to_data(float targetVoltage, float minVoltage, float maxVoltage, uint16_t resolution);
uint16_t ok_float_to_u16(float f, float min, float max);
uint8_t tim_get_capture_prescaler(TIM_HandleTypeDef *htim, uint32_t channel);
uint32_t tim_get_capture_period(TIM_HandleTypeDef *htim, uint32_t current, uint32_t previous);

template <typename T>
T ok_arr_average(T arr[], int n)
{
    // Find sum of array element
    T sum = 0;
    for (int i = 0; i < n; i++)
        sum += arr[i];

    return sum / n;
}

template <typename T>
constexpr T ok_clamp(T x, T min, T max)
{
    return x < min ? min : (x > max ? max : x);
}