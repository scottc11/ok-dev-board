#include "logger.h"

UART_HandleTypeDef huart3;
okQueue<char *> Q_logger(5);

void logger_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    gpio_enable_clock(OK_UART_RX);

    GPIO_InitStruct.Pin = gpio_get_pin(OK_UART_RX);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(gpio_get_port(OK_UART_RX), &GPIO_InitStruct);

    gpio_enable_clock(OK_UART_TX);

    GPIO_InitStruct.Pin = gpio_get_pin(OK_UART_TX);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(gpio_get_port(OK_UART_TX), &GPIO_InitStruct);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}

void logger_log(char const *str)
{
    uart_transmit((uint8_t *)str);
}

void logger_log(int const num) {
    char tmp[33];
    itoa(num, tmp, 10);
    uart_transmit((uint8_t *)tmp);
}

void logger_log(uint32_t const num) {
    char str[33];
    itoa(num, str, 10);
    uart_transmit((uint8_t *)str);
}

void logger_log(float const f) {
    char str[33];
    utoa(f, str, 10);
    // snprintf(str, sizeof(str), "%f", f);
    uart_transmit((uint8_t *)str);
}

void logger_log_err(char *str)
{
    uart_transmit((uint8_t *)str);
}

/**
 * @brief Hardware UART Transmit function
 * 
 * @param data 
 */
void uart_transmit(uint8_t *data)
{
    HAL_UART_Transmit(&huart3, data, strlen((const char *)data), HAL_MAX_DELAY);
}

void logger_log_system_config()
{
    logger_log("\n");
    logger_log("SYSCLK: ");
    logger_log(HAL_RCC_GetSysClockFreq());
    logger_log("\n");
    logger_log("HCLK: ");
    logger_log(HAL_RCC_GetHCLKFreq());
    logger_log("\n");
    logger_log("PCLK1: ");
    logger_log(HAL_RCC_GetPCLK1Freq());
    logger_log("\n");
    logger_log("PCLK2: ");
    logger_log(HAL_RCC_GetPCLK2Freq());
    logger_log("\n");
}

void logger_queue_message() {

}

void TASK_logger(void *params) {
    logger_init();
    logger_log_system_config();
    char * str;
    while (1)
    {

    }   
}