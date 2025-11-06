#include "task_controller.h"

void task_controller(void *params)
{
    GlobalControl *controller = (GlobalControl *)params;

    thController = xTaskGetCurrentTaskHandle();

    while (1)
    {
        // you could bit mask the first 16 bits to determine the command, and the bottom 16 bits to determine the channel to act on.
        uint32_t notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t channel = noti_get_channel(notification);
        CTRL_ACTION action = noti_get_command(notification);

        switch (action)
        {
        case CTRL_ACTION::ENTER_1VO_CALIBRATION:
            vTaskSuspend(main_task_handle); // this doesn't actually solve anything if controller.mode is "VCO_CALIBRATION"
            suspend_sequencer_task(); // suspend the sequencer task so that the clock doesn't advance while we are calibrating
            controller->clock->vcoFrequencyDetectionMode = true;
            HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4); // just to be safe before init
            controller->clock->enableInputCaptureISR();
            controller->calibrator.initVCOCalibration(controller->channels[channel]); // starts input capture again
            break;

        case CTRL_ACTION::EXIT_1VO_CALIBRATION:
            controller->saveCalibrationDataToFlash();
            controller->clock->vcoFrequencyDetectionMode = false;
            controller->clock->disableInputCaptureISR();
            controller->disableVCOCalibration();
            vTaskResume(main_task_handle);
            resume_sequencer_task();
            break;

        // this event is triggered from the input capture ISR once all capture readings have been collected.
        case CTRL_ACTION::HANDLE_CAPTURE_EVENT:
            controller->calibrator.handleInputCapture();
            break;
            
        case CTRL_ACTION::EXIT_BENDER_CALIBRATION:
            // do something
            break;

        case CTRL_ACTION::ADC_SAMPLING_PROGRESS:
            // channel -> which channel
            // data    -> value between 0..100
            // uint8_t channel = (uint8_t)bitwise_slice(notification, 8, 8);
            // uint16_t progress = (uint16_t)bitwise_slice(notification, 16, 16);
            break;

        case CTRL_ACTION::CONFIG_SAVE:
            suspend_sequencer_task();
            break;
        default:
            break;
        }
        logger_log_task_watermark(); // TODO: delete for production
    }
}