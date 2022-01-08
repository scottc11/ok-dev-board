#include "SuperClock.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

SuperClock *SuperClock::instance = NULL;

void SuperClock::start()
{
    HAL_StatusTypeDef status;
    status = HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
    error_handler(status);
    status = HAL_TIM_Base_Start_IT(&htim4);
    error_handler(status);
}

/**
 * @brief initialize TIM2 as a slave to TIM1
 * @param prescaler setting to 1 should be best
 * @param period setting to 65535 should be best
*/
void SuperClock::initTIM2(uint16_t prescaler, uint32_t period) // isn't TIM2 a 32-bit timer? This could greatly increase the resolution of frequency detection.
{
    __HAL_RCC_TIM2_CLK_ENABLE(); // turn on timer clock

    gpio_config_input_capture(EXT_CLOCK_INPUT); // config PA3 in input capture mode

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};
    HAL_StatusTypeDef status;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = prescaler;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = period;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    status = HAL_TIM_Base_Init(&htim2);
    if (status != HAL_OK)
        error_handler(status);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    status = HAL_TIM_IC_Init(&htim2);
    if (status != HAL_OK)
        error_handler(status);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    status = HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
    if (status != HAL_OK)
        error_handler(status);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;                     // dedicated prescaler allows to “slow down” the frequency of the input signal
    sConfigIC.ICFilter = 0;                                     // filter used to “debounce” the input signal
    status = HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
    if (status != HAL_OK)
        error_handler(status);
}

void SuperClock::initTIM4(uint16_t prescaler, uint16_t period)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = prescaler;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = period;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim4);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
}

void SuperClock::setFrequency(uint32_t freq) {
    if (freq < MAX_CLOCK_FREQ) {
        ticksPerStep = MAX_CLOCK_FREQ;
    } else if (freq > MIN_CLOCK_FREQ) {
        ticksPerStep = MIN_CLOCK_FREQ;
    } else {
        ticksPerStep = freq;
    }

    ticksPerPulse = ticksPerStep / (PPQN - 1); // Forget why we multiply by 2
}

/**
 * The key in this callback is to reset the sequencer and/or clock step to 0 (ie. step 1)
 * This way, the sequence will always be in sync with the beat because the rising edge of the external tempo signal will always line up to tick 0 of the sequence.
 * 
 * The caveat being we risk losing ticks near the end of each external clock signal, because the callback could execute before the final few ticks have a chance to be executed.
 * This could cause issues if there is important sequence data in those last remaining ticks, such as setting a trigger output high or low.
 * 
 * To remedy this problem, we could keep track of how many ticks have executed in the sequence for each clock signal, so that if the HAL_TIM_IC_CaptureCallback executes prior to 
 * the last tick in the sequence, we could quickly execute all those remaining ticks in rapid succession - which could sound weird, but likely neccessary.
 * 
 * NOTE: The downfall of this approach is that some steps of the sequence will be missed when the external tempo signal increases,
 * but this should be a more musical way to keeping things in sync.
 * 
 * NOTE: When jumping from a really fast tempo to a really slow tempo, the sequence steps will progress much faster than the incoming tempo (before the IC Calculation has time to update the sequencer pulse duration)
 * This means the sequence could technically get several beats ahead of any other gear.
 * To handle this, you could prevent the next sequence step from occuring if all sub-steps of the current step have been executed prior to a new IC event
 */
void SuperClock::handleInputCaptureCallback()
{
    pulse = 0;
    __HAL_TIM_SetCounter(&htim2, 0); // reset after each input capture
    __HAL_TIM_SetCounter(&htim4, 0); // reset after each input capture
    uint32_t inputCapture = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_4);
    uint16_t pulse = inputCapture / PPQN;
    __HAL_TIM_SetAutoreload(&htim4, pulse);
    this->handleOverflowCallback();
    // setFrequency(inputCapture);

    if (input_capture_callback) input_capture_callback();
}

/**
 * @brief this callback gets called everytime TIM4 overflows.
 * Increments pulse counter
*/ 
void SuperClock::handleOverflowCallback()
{
    if (ppqnCallback) ppqnCallback(pulse);

    if (pulse == 0) {
        if (resetCallback) resetCallback();
    }

    if (pulse < PPQN - 1) {
        pulse++;
    } else {
        pulse = 0;
    }
}

void SuperClock::attachInputCaptureCallback(Callback<void()> func)
{
    input_capture_callback = func;
}

void SuperClock::attachPPQNCallback(Callback<void(uint8_t pulse)> func)
{
    ppqnCallback = func;
}

void SuperClock::attachResetCallback(Callback<void()> func)
{
    resetCallback = func;
}

void SuperClock::RouteOverflowCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim4)
    {
        instance->handleOverflowCallback();
    }
}

void SuperClock::RouteCaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        instance->handleInputCaptureCallback();
    }
}

/**
  * @brief This function handles TIM2 global interrupt.
*/
extern "C" void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/**
  * @brief This function handles TIM4 global interrupt.
*/
extern "C" void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

/**
 * @brief This callback handles all TIMx overflow interupts (if TIMx was configured in interupt mode)
 * Ideally, you would use this to manage a global / system tick value, which then gets devided down
 * to handle events at a lower frequency.
*/
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    SuperClock::RouteOverflowCallback(htim);
}

/**
 * @brief Input Capture Callback for all TIMx configured in Input Capture mode
*/ 
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    SuperClock::RouteCaptureCallback(htim);
}