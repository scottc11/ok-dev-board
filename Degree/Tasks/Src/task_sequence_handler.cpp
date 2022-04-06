#include "task_sequence_handler.h"

TaskHandle_t sequencer_task_handle;
QueueHandle_t sequencer_queue;

/**
 * @brief Task which listens for a notification from the SuperClock
 *
 * @param params
 */
void task_sequence_handler(void *params)
{
    GlobalControl *ctrl = (GlobalControl *)params;
    sequencer_task_handle = xTaskGetCurrentTaskHandle();
    sequencer_queue = xQueueCreate(96, sizeof(uint32_t));
    uint32_t event = 0x0;
    while (1)
    {
        // queue == [advance, advance, advance, clear, advance, freeze, advance, advance ]
        xQueueReceive(sequencer_queue, &event, portMAX_DELAY);
        CHAN channel = (CHAN)bitwise_slice(event, 24, 8);
        SEQ action = (SEQ)bitwise_slice(event, 16, 8);
        uint16_t data = bitwise_slice(event, 0, 16);

        switch (action)
        {
        case SEQ::ADVANCE:
            ctrl->channels[channel]->sequence.advance();
            ctrl->channels[channel]->handleClock();
            break;
        case SEQ::FREEZE:
            if (channel == CHAN::ALL) {
                for (int i = 0; i < CHANNEL_COUNT; i++)
                    ctrl->channels[i]->freeze((bool)data);
            } else {
                ctrl->channels[channel]->freeze((bool)data);
            }
            break;
        case SEQ::RESET:
            if (channel == CHAN::ALL)
            {
                for (int i = 0; i < CHANNEL_COUNT; i++)
                    ctrl->channels[i]->resetSequence();
            } else {
                ctrl->channels[channel]->resetSequence();
            }
            break;
        case SEQ::CLEAR_TOUCH:
            if (channel == CHAN::ALL)
            {
                for (int i = 0; i < CHANNEL_COUNT; i++)
                {
                    ctrl->channels[i]->sequence.clearAllTouchEvents();
                }
            } else {
                ctrl->channels[channel]->sequence.clearAllTouchEvents();
            }
            break;
        case SEQ::CLEAR_BEND:
            if (channel == CHAN::ALL)
            {
                for (int i = 0; i < CHANNEL_COUNT; i++)
                {
                    ctrl->channels[i]->sequence.clearAllBendEvents();
                }
            }
            else
            {
                ctrl->channels[channel]->sequence.clearAllBendEvents();
            }
            break;
        case SEQ::RECORD_ENABLE:
            for (int i = 0; i < CHANNEL_COUNT; i++)
                ctrl->channels[i]->enableSequenceRecording();
            break;
        case SEQ::RECORD_DISABLE:
            for (int i = 0; i < CHANNEL_COUNT; i++)
                ctrl->channels[i]->disableSequenceRecording();
            break;
        case SEQ::SET_LENGTH:
            ctrl->channels[channel]->updateSequenceLength(data);
            break;
        }
    }
}

void dispatch_sequencer_event(CHAN channel, SEQ action, uint16_t position)
{
    // | chan | event | position |
    uint32_t event = ((uint8_t)channel << 24) | ((uint8_t)action << 16) | position;
    xQueueSend(sequencer_queue, &event, portMAX_DELAY);
}

void dispatch_sequencer_event_ISR(CHAN channel, SEQ action, uint16_t position)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // | chan | event | position |
    uint32_t event = ((uint8_t)channel << 24) | ((uint8_t)action << 16) | position;
    xQueueSendFromISR(sequencer_queue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}