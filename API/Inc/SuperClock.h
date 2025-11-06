#pragma once

#include "common.h"
#include "logger.h"
#include "tim_api.h"
#include "Callback.h"
#include "Algorithms.h"

#ifndef PPQN
#define PPQN 96
#endif

#define PPQN_ERROR (PPQN - (PPQN / 6))

#ifndef EXT_CLOCK_INPUT
#define EXT_CLOCK_INPUT PA_3
#endif

#define MAX_TICKS_PER_PULSE 34299  // (40 BPM)  MAX TIM4 tickers per pulse
#define MIN_TICKS_PER_PULSE 5716   // (240 BPM) MIN TIM4 tickers per pulse

extern TIM_HandleTypeDef htim2; // 32-bit timer
extern TIM_HandleTypeDef htim4; // 16-bit timer

extern "C" void TIM2_IRQHandler(void);
extern "C" void TIM4_IRQHandler(void);

class SuperClock {
public:

    int pulse;              // the current PPQN
    uint16_t ticksPerStep;  // how many TIM2 ticks per one step / quarter note
    uint16_t ticksPerPulse; // how many TIM2 ticks for one PPQN
    bool externalInputMode;
    bool vcoFrequencyDetectionMode;

    Callback<void()> tickCallback;           // this callback gets executed at a frequency equal to tim1_freq
    Callback<void()> input_capture_callback; // this callback gets executed every on the rising edge of external input
    Callback<void(uint8_t pulse)> ppqnCallback; // this callback gets executed at a rate equal to input capture / PPQN. It passes the current tick values as arguments
    Callback<void(uint8_t pulse)> resetCallback;
    Callback<void()> overflowCallback;       // callback executes when all when a full step completes

    /**
     * TODO: initial inputCapture value should be the product of TIM1 and TIM2 prescaler values combined with 120 BPM
     * so that the sequencer always gets initialized at 120 bpm, no matter the speed of the timers
    */
    SuperClock()
    {
        instance = this;
        ticksPerStep = 11129;
        ticksPerPulse = ticksPerStep / PPQN;
    };

    void init();
    void initTIM2(uint16_t prescaler, uint32_t period);
    void initTIM4(uint16_t prescaler, uint16_t period);
    void start();

    void setPulseFrequency(uint32_t ticks);
    uint16_t convertADCReadToTicks(uint16_t min, uint16_t max, uint16_t value);
    void enableInputCaptureISR();
    void disableInputCaptureISR();

    // Callback Setters
    void attachInputCaptureCallback(Callback<void()> func);
    void attachPPQNCallback(Callback<void(uint8_t pulse)> func);
    void attachResetCallback(Callback<void(uint8_t pulse)> func);
    
    // Low Level HAL interupt handlers
    void handleInputCaptureCallback();
    void handleOverflowCallback();
    static void RouteOverflowCallback(TIM_HandleTypeDef *htim);
    static void RouteCaptureCallback(TIM_HandleTypeDef *htim);

private:
    static SuperClock *instance;
};