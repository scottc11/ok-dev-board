#include "main.h"
#include "SuperClock.h"
#include "DigitalOut.h"
#include "InterruptIn.h"
#include "Callback.h"
#include "MultiChanADC.h"
#include "SX1509.h"
#include "I2C.h"
#include "MPR121.h"
#include "MCP23017.h"
#include "DAC8554.h"
#include "Flash.h"
#include "TouchChannel.h"
#include "Degrees.h"
#include "GlobalControl.h"

using namespace DEGREE;

PinName adcPins[8] = {ADC_A, ADC_B, ADC_C, ADC_D, PB_ADC_A, PB_ADC_B, PB_ADC_C, PB_ADC_D};

I2C i2c1(I2C1_SDA, I2C1_SCL, I2C::Instance::I2C_1);
I2C i2c3(I2C3_SDA, I2C3_SCL, I2C::Instance::I2C_3);

DAC8554 dac1(SPI2_MOSI, SPI2_SCK, DAC1_CS);

MCP23017 toggleSwitches(&i2c3, MCP23017_DEGREES_ADDR);

MPR121 touchA(&i2c1, TOUCH_INT_A);
MPR121 touchB(&i2c1, TOUCH_INT_B, MPR121::ADDR_VDD);
MPR121 touchC(&i2c1, TOUCH_INT_C, MPR121::ADDR_SCL);
MPR121 touchD(&i2c1, TOUCH_INT_D, MPR121::ADDR_SDA);

SX1509 ledsA(&i2c3, SX1509_CHAN_A_ADDR);
SX1509 ledsB(&i2c3, SX1509_CHAN_B_ADDR);
SX1509 ledsC(&i2c3, SX1509_CHAN_C_ADDR);
SX1509 ledsD(&i2c3, SX1509_CHAN_D_ADDR);

SuperClock superClock;

uint16_t FILTERED_ADC_VALUES[ADC_DMA_BUFF_SIZE];

Degrees degrees(DEGREES_INT, &toggleSwitches);

TouchChannel chanA(&touchA, &ledsA, &degrees, &dac1, DAC8554::CHAN_A, 4);
TouchChannel chanB(&touchB, &ledsB, &degrees, &dac1, DAC8554::CHAN_B, 5);
TouchChannel chanC(&touchC, &ledsC, &degrees, &dac1, DAC8554::CHAN_C, 6);
TouchChannel chanD(&touchD, &ledsD, &degrees, &dac1, DAC8554::CHAN_D, 7);

GlobalControl glblCtrl(&chanA, &chanB, &chanC, &chanD, &degrees);

/**
 * @brief handle all ADC inputs here
*/ 
void ADC1_DMA_Callback(uint16_t values[])
{
  for(int i=0; i < ADC_DMA_BUFF_SIZE; i++)
  {
    // take the raw adc values array and chuck them into a filtered adc array
    FILTERED_ADC_VALUES[i] = (convert12to16(values[i]) * 0.1) + (FILTERED_ADC_VALUES[i] * (1 - 0.1));
  }
}

DigitalOut led(TEMPO_LED);
void toggleLED() {
  led = !led.read();
}

// ----------------------------------------
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  superClock.initTIM1(16, 100);
  superClock.initTIM2(1, 65535);
  superClock.attachInputCaptureCallback(callback(toggleLED));
  superClock.start();

  multi_chan_adc_init();
  multi_chan_adc_start();

  i2c1.init();
  i2c3.init();

  glblCtrl.init();

  while (1)
  {
    glblCtrl.poll();
  }
}
// ----------------------------------------