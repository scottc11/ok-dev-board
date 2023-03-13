#pragma once

#include "system_clock_config.h"
#include "cmsis_os.h"
#include "task_handles.h"

#include "common.h"

// #define LOGGING_ENABLED
#define BOARD_VERSION 42

#ifndef FIRMWARE_VERSION // this will be passed in as a flag by compiler
#define FIRMWARE_VERSION "default"
#endif

#define PPQN 96
#define PPQN_DIV_2 (PPQN / 2)
#define PPQN_DIV_4 (PPQN / 4)
#define PPQN_DIV_8 (PPQN / 8)

#define DEFAULT_SEQ_LENGTH 8
#define MAX_SEQ_LENGTH 32
#define MAX_SEQ_LENGTH_PPQN (MAX_SEQ_LENGTH * PPQN)

#define CHANNEL_COUNT 4
#define ADC_DMA_BUFF_SIZE   9
#define ADC_TIM_PRESCALER   100
#define ADC_TIM_PERIOD      2000

#define DAC_1VO_ARR_SIZE 72
#define FIRMWARE_VERSION_SIZE 8

#define SETTINGS_FIRMWARE_VERSION 0
#define SETTINGS_DAC_1VO FIRMWARE_VERSION_SIZE
#define SETTINGS_BENDER_MIN (SETTINGS_DAC_1VO + DAC_1VO_ARR_SIZE)
#define SETTINGS_BENDER_MAX (SETTINGS_BENDER_MIN + 1)
#define SETTINGS_BENDER_MODE (SETTINGS_BENDER_MAX + 1)
#define SETTINGS_PITCH_BEND_RANGE (SETTINGS_BENDER_MODE + 1)
#define SETTINGS_SEQ_LENGTH (SETTINGS_PITCH_BEND_RANGE + 1)
#define SETTINGS_QUANTIZE_AMOUNT (SETTINGS_SEQ_LENGTH + 1)

#define CALIBRATION_ARR_SIZE (SETTINGS_QUANTIZE_AMOUNT + 1)
#define SETTINGS_BUFFER_SIZE (CALIBRATION_ARR_SIZE * CHANNEL_COUNT)

#define FLASH_CONFIG_ADDR ADDR_FLASH_SECTOR_7

#define FLASH_CHANNEL_CONFIG_SIZE 32 // 32 bytes = 8 32-bit words (one row in flash)
#define FLASH_SEQUENCE_CONFIG_SIZE 32
#define FLASH_SEQUENCE_DATA_SIZE MAX_SEQ_LENGTH_PPQN

#define FLASH_CHANNEL_CONFIG_ADDR (uint32_t)0x08060600
#define FLASH_SEQUENCE_CONFIG_ADDR (FLASH_CHANNEL_CONFIG_ADDR + FLASH_CHANNEL_CONFIG_SIZE)
#define FLASH_SEQUENCE_DATA_ADDR (FLASH_SEQUENCE_CONFIG_ADDR + FLASH_SEQUENCE_CONFIG_SIZE)
#define FLASH_CHANNEL_BLOCK_SIZE ((FLASH_SEQUENCE_DATA_ADDR + FLASH_SEQUENCE_DATA_SIZE) - FLASH_CHANNEL_CONFIG_ADDR + (uint32_t)0x1000)

#define OCTAVE_COUNT 4
#define DEGREE_COUNT 8

#define BENDER_DAC_ZERO        32767
#define BENDER_NOISE_THRESHOLD 1000

#define DISPLAY_MAX_CURRENT    127   // for managing current

#define REC_LED PC_13
#define FREEZE_LED PB_7
#define TEMPO_LED PA_1
#define TEMPO_POT PA_2
#define TEMPO_POT_MIN_ADC 1000
#define TEMPO_POT_MAX_ADC 60000

#define EXT_CLOCK_INPUT PA_3

#if BOARD_VERSION == 38
#define INT_CLOCK_OUTPUT PB_2
#else
#define INT_CLOCK_OUTPUT PA_0
#endif

#define ADC_A PA_6
#define ADC_B PA_7
#define ADC_C PC_5
#define ADC_D PC_4

#define PB_ADC_A PA_4
#define PB_ADC_B PA_5
#define PB_ADC_C PB_0
#define PB_ADC_D PB_1

#define GATE_OUT_A PC_2
#define GATE_OUT_B PC_3
#define GATE_OUT_C PC_7
#define GATE_OUT_D PC_6

#define GLOBAL_GATE_OUT PB_10

#define I2C3_SDA PC_9
#define I2C3_SCL PA_8
#define I2C1_SDA PB_9
#define I2C1_SCL PB_8

#define TOUCH_INT_A PC_1
#define TOUCH_INT_B PC_0
#define TOUCH_INT_C PC_15
#define TOUCH_INT_D PC_14

#define DEGREES_INT PB_4

#define BUTTONS_INT    PB_5
#define GLBL_TOUCH_INT PB_6

#define DAC1_CS PB_12
#define DAC2_CS PC_12

#define SPI2_MOSI PB_15
#define SPI2_MISO PB_14
#define SPI2_SCK  PB_13

#define UART_RX PC_11
#define UART_TX PC_10

#define MCP23017_CTRL_ADDR 0x24 // 0100100

#define SX1509_CHAN_A_ADDR 0x3E
#define SX1509_CHAN_B_ADDR 0x70
#define SX1509_CHAN_C_ADDR 0x3F
#define SX1509_CHAN_D_ADDR 0x71

#define CAP1208_ADDR 0x50 // 0010100

#define MCP23017_DEGREES_ADDR 0x20 // 0100000
#define MCP23017_CTRL_ADDR    0x24    // 0100100

    enum CHAN {
        A,
        B,
        C,
        D,
        ALL
    };
typedef enum CHAN CHAN;

#define ISR_ID_TOGGLE_SWITCHES 0
#define ISR_ID_TACTILE_BUTTONS 1
#define ISR_ID_TOUCH_PADS 2