#include "AnalogHandle.h"

SemaphoreHandle_t AnalogHandle::semaphore;
AnalogHandle *AnalogHandle::_instances[ADC_DMA_BUFF_SIZE] = {0};

AnalogHandle::AnalogHandle(PinName pin)
{
    // iterate over static member ADC_PINS and match index to pin
    for (int i = 0; i < ADC_DMA_BUFF_SIZE; i++)
    {
        if (pin == ADC_PINS[i])
        {
            index = i;
            break;
        }
    }
    // Add constructed instance to the static list of instances (required for IRQ routing)
    for (int i = 0; i < ADC_DMA_BUFF_SIZE; i++)
    {
        if (_instances[i] == NULL)
        {
            _instances[i] = this;
            break;
        }
    }
}

uint16_t AnalogHandle::read_u16()
{
    return invert ? BIT_MAX_16 - currValue : currValue;
}

void AnalogHandle::invertReadings()
{
    this->invert = !this->invert;
}

void AnalogHandle::setFilter(float value)
{
    if (value == 0)
    {
        filter = false;
    }
    else if (value > (float)1)
    {
        // raise error
        filter = false;
    }
    else
    {
        prevValue = convert12to16(DMA_BUFFER[index]);
        filterAmount = value;
        filter = true;
    }
}

SemaphoreHandle_t* AnalogHandle::initDenoising() {
    this->disableFilter();
    this->denoising = true;
    // create a semaphore
    denoiseSemaphore = xSemaphoreCreateBinary();
    return &denoiseSemaphore;
}

// set this as a task so that in the main loop you block with a semaphore until this task gives the semaphore back (once it has completed)
void AnalogHandle::calculateSignalNoise(uint16_t sample)
{
    // get max read, get min read, get avg read
    if (sampleCounter < ADC_SAMPLE_COUNTER_LIMIT)
    {
        if (sampleCounter == 0) {
            noiseCeiling = sample;
            noiseFloor = sample;
        } else {
            if (sample > noiseCeiling)
            {
                noiseCeiling = sample;
            }
            else if (sample < noiseFloor)
            {
                noiseFloor = sample;
            }
        }
        sampleCounter++;
    }
    else
    {
        denoising = false;
        sampleCounter = 0; // reset back to 0 for later use
        idleNoiseThreshold = (noiseCeiling - noiseFloor) / 2;
        xSemaphoreGive(denoiseSemaphore);
    }
}

void AnalogHandle::sampleReadyCallback(uint16_t sample)
{
    prevValue = currValue;
    currValue = convert12to16(sample);
    if (this->denoising) {
        this->calculateSignalNoise(currValue);
    }
    if (filter) {
        currValue = (currValue * filterAmount) + (prevValue * (1 - filterAmount));
    }
    // set value
}

void AnalogHandle::RouteConversionCompleteCallback() // static
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(AnalogHandle::semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void AnalogHandle::sampleReadyTask(void *params) {
    while (1)
    {
        xSemaphoreTake(AnalogHandle::semaphore, portMAX_DELAY);
        for (auto ins : _instances)
        {
            if (ins) // if instance not NULL
            {
                ins->sampleReadyCallback(AnalogHandle::DMA_BUFFER[ins->index]);
            }
        }
    }
}

void AnalogHandle::log_noise_threshold_to_console(char * source_str)
{
    logger_log(source_str);
    logger_log(" ADC Noise: ");
    logger_log(this->idleNoiseThreshold);
    logger_log("\n");
}