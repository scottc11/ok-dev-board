#include "task_handles.h"

TaskHandle_t main_task_handle;
TaskHandle_t thStartCalibration;
TaskHandle_t thExitCalibration;
TaskHandle_t thCalibrate;
TaskHandle_t thController;

CTRL_ACTION noti_get_command(uint32_t notification)
{
    return (CTRL_ACTION)bitwise_slice(notification, 0, 8);
}

uint8_t noti_get_channel(uint32_t notification)
{
    return (uint8_t)bitwise_slice(notification, 8, 8);
}

uint16_t noti_get_data(uint32_t notification) {
    return (uint16_t)bitwise_slice(notification, 16, 16);
}

/**
 * @brief dispatch an action to the controller task
 * | data | data | channel | action |
 * @param action
 * @param channel
 * @param data
 */
void ctrl_dispatch(CTRL_ACTION action, uint8_t channel, uint16_t data) {
    uint32_t dispatch = (data << 16) | (channel << 8) | (action);
    xTaskNotify(thController, dispatch, eSetValueWithOverwrite);
}

void ctrl_dispatch_ISR(CTRL_ACTION action, uint8_t channel, uint16_t data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t dispatch = (data << 16) | (channel << 8) | (action);
    xTaskNotifyFromISR(thController, dispatch, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}