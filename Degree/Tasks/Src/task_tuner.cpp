#include "task_tuner.h"

static const float TARGET_FREQUENCY_C1 = PITCH_FREQ_ARR[0];
QueueHandle_t tuner_queue;
TaskHandle_t tuner_task_handle;

void timer_callback() {
    xTaskNotify(thController, CTRL_CMNDS::EXIT_VCO_TUNING, eSetValueWithoutOverwrite);
}

void task_tuner(void *params)
{
    TouchChannel *channel = (TouchChannel *)params;
    float frequency;
    tuner_queue = xQueueCreate(1, sizeof(float));
    SoftwareTimer timer(timer_callback, 3000, false);
    channel->output.resetDAC();
    while (1)
    {
        // listen for items on queue
        xQueueReceive(tuner_queue, &frequency, portMAX_DELAY);
        channel->display->clear(channel->channelIndex);
        // TODO: determine the distance incoming frequency is relative to target freq and DIM the leds based on this value
        if (frequency > TARGET_FREQUENCY_C1 + TUNING_TOLERANCE)
        {
            channel->display->setChannelLED(channel->channelIndex, 0, true);
            channel->display->setChannelLED(channel->channelIndex, 1, true);
            channel->display->setChannelLED(channel->channelIndex, 2, true);
            channel->display->setChannelLED(channel->channelIndex, 3, true);
            timer.reset();
        }
        else if (frequency < TARGET_FREQUENCY_C1 - TUNING_TOLERANCE)
        {
            channel->display->setChannelLED(channel->channelIndex, 12, true);
            channel->display->setChannelLED(channel->channelIndex, 13, true);
            channel->display->setChannelLED(channel->channelIndex, 14, true);
            channel->display->setChannelLED(channel->channelIndex, 15, true);
            timer.reset();
        }
        else if (frequency > TARGET_FREQUENCY_C1 - TUNING_TOLERANCE && frequency < TARGET_FREQUENCY_C1 + TUNING_TOLERANCE)
        {
            
            channel->display->setChannelLED(channel->channelIndex, 4, true);
            channel->display->setChannelLED(channel->channelIndex, 5, true);
            channel->display->setChannelLED(channel->channelIndex, 6, true);
            channel->display->setChannelLED(channel->channelIndex, 7, true);
            channel->display->setChannelLED(channel->channelIndex, 8, true);
            channel->display->setChannelLED(channel->channelIndex, 9, true);
            channel->display->setChannelLED(channel->channelIndex, 10, true);
            channel->display->setChannelLED(channel->channelIndex, 11, true);
            // start timer
            timer.start();
        }
    }
}