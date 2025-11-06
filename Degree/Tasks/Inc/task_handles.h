#pragma once

#include "main.h"
#include "BitwiseMethods.h"

#define TUNING_TOLERANCE 0.4f // tolerable frequency tuning difference

extern TaskHandle_t main_task_handle;
extern TaskHandle_t thStartCalibration;
extern TaskHandle_t thExitCalibration;
extern TaskHandle_t thCalibrate;
extern TaskHandle_t thController;
extern TaskHandle_t tuner_task_handle;
extern TaskHandle_t thInterruptHandler;

extern QueueHandle_t qhInterruptQueue;
extern QueueHandle_t tuner_queue;
extern QueueHandle_t qh_adc_sample_ready;

enum CTRL_ACTION
{
    ENTER_1VO_CALIBRATION = 0,
    EXIT_1VO_CALIBRATION = 1,
    EXIT_BENDER_CALIBRATION = 2,
    ENTER_VCO_TUNING = 3,
    EXIT_VCO_TUNING = 4,
    HANDLE_CAPTURE_EVENT = 5,
    ADC_SAMPLING_PROGRESS = 6,
    CONFIG_SAVE,
    CONFIG_RESET
};
typedef enum CTRL_ACTION CTRL_ACTION;

CTRL_ACTION noti_get_command(uint32_t notification);
uint8_t noti_get_channel(uint32_t notification);

void ctrl_dispatch(CTRL_ACTION action, uint8_t channel, uint16_t data);
void ctrl_dispatch_ISR(CTRL_ACTION action, uint8_t channel, uint16_t data);