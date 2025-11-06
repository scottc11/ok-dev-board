#include "Calibrator.h"
#include "TouchChannel.h"

/**
 * @brief This at a very low level gets called by the SuperClock instance when an input capture event occurs, assuming the SuperClock is in VCO frequency detection mode.
 * 
 * It gets called by the Controller class via a callback function (passed into SuperClock instance).
 * 
 * 
 * 
 * @param htim 
 */
void Calibrator::captureCallback()
{
    static int capture_index = 0;
    
    // store the capture reading (note: this is not the period, but the capture value, you will have to get the difference between the current and previous capture value to get the period)
    CAPTURE_READINGS[capture_index] = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_4);

    capture_index++;

    // Once enough readings have been collected move to processing the readings
    if (capture_index >= NUM_CAPTURE_READINGS)
    {
        capture_index = 0;
        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);
        ctrl_dispatch_ISR(CTRL_ACTION::HANDLE_CAPTURE_EVENT, 0, 0);
    }
}

/**
 * @brief Use exponential interpolation to populate a tuning table based on the detected frequencies of the
 * VCO at varying CV output levels.
 *
 * @note The idea here is, if we detect the frequency of the VCO when the DAC is at its lowest voltage
 * value, and then detect the VCO frequency at the DACs highest voltage value ( with 30 or 40 points in-between ),
 * then we can use interpolation to calculate the precise DAC values needed to obtain specific VCO frequencies.
 */
void Calibrator::initVCOCalibration(DEGREE::TouchChannel *targetChannel)
{
    this->channel = targetChannel;

    // I think to start, just try calibrating the VCO from 0V to 5V (5 octaves)
    calibrationIndex = 0;
    uint16_t dac_floor = ok_voltage_to_data(V_OUT_MIN, V_OUT_MIN, V_OUT_MAX, DAC_RESOLUTION); // lowest possible voltage pre calibration
    uint16_t dac_ceiling = ok_voltage_to_data(V_OUT_MAX, V_OUT_MIN, V_OUT_MAX, DAC_RESOLUTION); // highest possible voltage pre calibration
    uint16_t stepSize = (dac_ceiling - dac_floor) / NUM_POINTS_INTERPOLATION;

    // generate DAC values which range from our DAC floor to our DAC ceiling, in equal steps
    for (int i = 0; i < NUM_POINTS_INTERPOLATION; i++)
    {
        this->interpolationData[i].dacValue = dac_floor + (i * stepSize);
    }

    // set the DAC to the first value in our interpolation data
    channel->output.dac->write(channel->output.dacChannel, this->interpolationData[calibrationIndex].dacValue);
    vTaskDelay(pdMS_TO_TICKS(100)); // wait for the DAC to settle

    // start the input capture
    // __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SET_ICPRESCALER(&htim2, TIM_CHANNEL_4, TIM_ICPSC_DIV1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4); // this will collect an input capture and then queue an event via RTOS (which then calls handleInputCapture())
}

/**
 * @brief Handle the input capture event (ISR will queue an event via RTOS)
 *
 * The capture value will be found in the array CAPTURE_READINGS[]. We can either take the average of the
 * last 8 captures, or we can take a single capture. A single capture appears to be quite accurate, but using more is safer.
 */
void Calibrator::handleInputCapture()
{
    float frequencyReadings[NUM_CAPTURE_READINGS - 1]; // One less than capture readings since we calculate periods between consecutive captures
    uint32_t capturePrescaler = tim_get_capture_prescaler(&htim2, TIM_CHANNEL_4);
    uint16_t prescaler = htim2.Init.Prescaler;
    uint32_t APBx_freq = tim_get_APBx_freq(&htim2);

    // Convert the raw capture timestamps to period values by taking differences between consecutive captures
    uint32_t periodReadings[NUM_CAPTURE_READINGS - 1]; // N captures yield N-1 periods between them

    // Calculate the time period between each pair of consecutive capture events
    for (int i = 0; i < NUM_CAPTURE_READINGS - 1; i++)
    {
        // Get the time difference between capture[i+1] and capture[i] to determine one signal period
        periodReadings[i] = tim_get_capture_period(&htim2, CAPTURE_READINGS[i + 1], CAPTURE_READINGS[i]);
    }

    // Convert each period measurement to frequency
    for (int i = 0; i < NUM_CAPTURE_READINGS - 1; i++)
    {
        uint32_t period = periodReadings[i] / capturePrescaler;
        frequencyReadings[i] = static_cast<float>(APBx_freq) / (float)((period * (prescaler + 1)));
    }

    // Average all the frequency measurements for a more accurate result
    float frequency = ok_arr_average<float>(frequencyReadings, NUM_CAPTURE_READINGS - 1);
    frequency = frequency / 2.0f; // no idea why this is needed, but it is. 

    // store the frequency in the calibration data
    this->interpolationData[calibrationIndex].frequency = frequency;

    // increment the calibration index
    calibrationIndex++;

    // if we have not yet reached the end of the calibration data, continue
    if (calibrationIndex < NUM_POINTS_INTERPOLATION)
    {
        if (calibrationIndex > 20) // increase capture prescaler for better accuracy at higher frequencies
        {
            __HAL_TIM_SET_ICPRESCALER(&htim2, TIM_CHANNEL_4, TIM_ICPSC_DIV8);
        }

        // NOTE: use mathematical extrapolation to estimate the remaining frequency values ?

        // set the DAC to the next value in our interpolation data
        channel->output.dac->write(channel->output.dacChannel, interpolationData[calibrationIndex].dacValue); // set DAC value for next frequency capture
        vTaskDelay(pdMS_TO_TICKS(3)); // wait for the DAC to settle

        // re-enable the input capture to collect the next frequency reading
        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
    }
    else
    {
        // if the first element is greater then the second element, a bad reading was taken and we need to restart
        if (interpolationData[0].frequency > interpolationData[1].frequency)
        {
            initVCOCalibration(channel);
            return;
        }

        // apply linear interpolation to the calibration data
        this->interpolateTuningTable();
        
        // exit the calibration process
        ctrl_dispatch(CTRL_ACTION::EXIT_1VO_CALIBRATION, 0, 0);
    }
}


/**
 * @brief Interpolate the tuning table for a specific channel
 * 
 * This function performs exponential interpolation on the calibration data to generate a tuning table
 * for a specific channel. It takes the target frequency and finds the corresponding DAC value that
 * will produce that frequency.
 */
void Calibrator::interpolateTuningTable()
{
    // the lowest note the target oscillator can acheive (at 0V) is where you should start the interpolation
    // so find the index of the first element in the calibrationData array that is greater than or equal to the target frequency
    int startIndex = 0;
    for (int j = 0; j < DAC_1VO_ARR_SIZE; j++)
    {
        if (this->interpolationData[0].frequency <= NOTE_FREQ_MAP[j])
        {
            startIndex = j;
            break;
        }
    }

    // Loop through each frequency in NOTE_FREQ_MAP
    for (int i = 0; i < DAC_1VO_ARR_SIZE; i++)
    {
        // you need to determine the lowest note the target ocsillator can acheive , rounded up, when the DAC is at its lowest value
        // add a check to see if 8 octaves can be achieved (ex. NOTE_FREQ_MAP[i] * 2^8)
        float targetFreq = NOTE_FREQ_MAP[startIndex + i]; // needs to be startIndex + i, but also NOTE_FREQ_MAP needs to be a longer array (add a few more octaves should be fine)

        // Find the interval in the calibrationTable that contains the target frequency
        int lowerIndex = 0;
        int upperIndex = 0;
        for (int j = 0; j < NUM_POINTS_INTERPOLATION - 1; j++)
        {
            if (this->interpolationData[j].frequency <= targetFreq && this->interpolationData[j + 1].frequency >= targetFreq)
            {
                lowerIndex = j;
                upperIndex = j + 1;
                break;
            }
        }

        // Perform exponential interpolation 🤓
        float freqLower = this->interpolationData[lowerIndex].frequency;
        float freqUpper = this->interpolationData[upperIndex].frequency;
        uint32_t dacLower = this->interpolationData[lowerIndex].dacValue;
        uint32_t dacUpper = this->interpolationData[upperIndex].dacValue;

        // Calculate the logarithmic interpolation factor
        float logFreqLower = std::log(freqLower);
        float logFreqUpper = std::log(freqUpper);
        float logTargetFreq = std::log(targetFreq);

        float interpolationFactor = (logTargetFreq - logFreqLower) / (logFreqUpper - logFreqLower);

        // Calculate the interpolated DAC value
        float logDacLower = std::log(static_cast<float>(dacLower));
        float logDacUpper = std::log(static_cast<float>(dacUpper));

        float logInterpolatedDac = logDacLower + interpolationFactor * (logDacUpper - logDacLower);
        float in = std::exp(logInterpolatedDac);
        uint16_t out = ok_float_to_u16(in, 0.0f, 65535.0f);

        // Store the interpolated DAC value in the tuning table
        channel->output.dacVoltageMap[i] = out;
    }
}

/**
 * @brief Store the calibration data (dacVoltageMap) in EEPROM for a specific channel
 */
void Calibrator::storeCalibrationToEEPROM(DEGREE::TouchChannel *targetChannel)
{
    if (targetChannel->channelIndex < 0 || targetChannel->channelIndex >= CHANNEL_COUNT) {
        return; // Invalid channel index or no channel set
    }
}

/**
 * @brief Load the calibration data from EEPROM for a specific channel
 */
void Calibrator::loadVCOCalDataFromEEPROM(DEGREE::TouchChannel *targetChannel)
{
    if (!targetChannel || targetChannel->channelIndex < 0 || targetChannel->channelIndex >= CHANNEL_COUNT) {
        return; // Invalid channel or channel index
    }
}

/**
 * @brief Clear the calibration data from EEPROM for a specific channel
 */
void Calibrator::clearCalibrationFromEEPROM(DEGREE::TouchChannel *targetChannel)
{
    if (!targetChannel || targetChannel->channelIndex < 0 || targetChannel->channelIndex >= CHANNEL_COUNT) {
        return; // Invalid channel or channel index
    }
}


/**
 * @brief convert a floating-point value to a uint16_t value
 *
 * @note 👀 ok-STM32F4/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c
 *
 * @param f
 * @param min
 * @param max
 * @return uint16_t
 */
uint16_t ok_float_to_u16(float f, float min, float max)
{
    // Scale the floating-point value to fit within the range of uint16_t
    float scaledValue = (f - min) / (max - min) * 65535.0f;

    // Clamp the scaled value to the range of uint16_t
    scaledValue = ok_clamp<float>(scaledValue, 0.0f, 65535.0f);

    // Convert the scaled value to uint16_t
    return (uint16_t)scaledValue;
}

/**
 * @brief convert a target voltage to a DAC value
 *
 * @param targetVoltage target voltage to output on DAC
 * @param minVoltage minimum value the DAC can output
 * @param maxVoltage maximum value the DAC can output
 * @param resolution DAC resolution ex. 4096, 65536
 * @return uint16_t
 */
uint16_t ok_voltage_to_data(float targetVoltage, float minVoltage, float maxVoltage, uint16_t resolution)
{
    float totalVoltageRange = maxVoltage - minVoltage;
    float voltageStepPerDacValue = totalVoltageRange / (resolution - 1);
    uint16_t dacValue = (targetVoltage - minVoltage) / voltageStepPerDacValue;
    return dacValue;
}

/**
 * @brief Get the amount of rising/falling edges to count before triggering the input capture interrupt
 *
 * @param htim
 * @param channel
 *
 * @return uint8_t how many events before the capture is performed
 */
uint8_t tim_get_capture_prescaler(TIM_HandleTypeDef *htim, uint32_t channel)
{
    uint32_t prescaler = __HAL_TIM_GetICPrescaler(htim, channel);
    switch (prescaler)
    {
    case TIM_ICPSC_DIV1:
        return 1;
    case TIM_ICPSC_DIV2:
        return 2;
    case TIM_ICPSC_DIV4:
        return 4;
    case TIM_ICPSC_DIV8:
        return 8;
    default:
        return 1;
    }
}

/**
 * @brief Calculate the period between two capture readings, handling the overflow case
 *
 * @param htim pointer to the timer handle
 * @param current current capture value
 * @param previous previous capture value
 * @return uint32_t period
 */
uint32_t tim_get_capture_period(TIM_HandleTypeDef *htim, uint32_t current, uint32_t previous)
{
    // If no overflow occurred
    if (current >= previous)
    {
        return current - previous;
    }
    // If overflow occurred
    else
    {
        return (htim->Instance->ARR - previous) + current + 1;
    }
}