##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.13.0-B3] date: [Sat Aug 28 14:32:18 EDT 2021] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ------------------------------------------------

######################################
# target
######################################
TARGET = ok-dev-board

FLASH_SIZE = $$((512 * 1024)) # 512 kB
RAM_SIZE = $$((128 * 1024)) # 128 kB

######################################
# building variables
######################################
# debug build?
DEBUG = 1

SERIAL_DEBUG ?= 0

# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
middleware/FreeRTOS/Source/croutine.c \
middleware/FreeRTOS/Source/event_groups.c \
middleware/FreeRTOS/Source/list.c \
middleware/FreeRTOS/Source/queue.c \
middleware/FreeRTOS/Source/stream_buffer.c \
middleware/FreeRTOS/Source/tasks.c \
middleware/FreeRTOS/Source/timers.c \
middleware/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
middleware/FreeRTOS/Source/portable/MemMang/heap_4.c \
middleware/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
System/Src/freertos.c \
System/Src/stm32f4xx_hal_timebase_tim.c \
System/Src/stm32f4xx_hal_msp.c \
System/Src/stm32f4xx_it.c \
System/Src/system_stm32f4xx.c \
System/Src/system_clock_config.c

CPP_SOURCES = \
API/Src/gpio_api.cpp \
API/Src/error_handler.cpp \
API/Src/Flash.cpp \
API/Src/I2C.cpp \
API/Src/InterruptIn.cpp \
API/Src/logger.cpp \
API/Src/SPI.cpp \
API/Src/DigitalIn.cpp \
API/Src/DigitalOut.cpp \
API/Src/SuperClock.cpp \
API/Src/tim_api.cpp \
API/rtos/Src/SoftwareTimer.cpp \
API/rtos/Src/Mutex.cpp \
Degree/Src/AnalogHandle.cpp \
Degree/Src/main.cpp \
Degree/Src/Bender.cpp \
Degree/Src/Display.cpp \
Degree/Src/MultiChanADC.cpp \
Degree/Src/SuperSeq.cpp \
Degree/Src/TouchChannel.cpp \
Degree/Src/GlobalControl.cpp \
Degree/Src/VoltPerOctave.cpp \
Degree/Tasks/Src/task_calibration.cpp \
Degree/Tasks/Src/task_controller.cpp \
Degree/Tasks/Src/task_handles.cpp \
Degree/Tasks/Src/task_tuner.cpp \
ok-drivers/drivers/CAP1208/CAP1208.cpp \
ok-drivers/drivers/DAC8554/DAC8554.cpp \
ok-drivers/drivers/SX1509/SX1509.cpp \
ok-drivers/drivers/IS31FL3739/IS31FL3739.cpp \
ok-drivers/drivers/MPR121/MPR121.cpp \
ok-drivers/drivers/MCP23017/MCP23017.cpp \
ok-drivers/utils/Algorithms/Algorithms.cpp \
ok-drivers/utils/ArrayMethods/ArrayMethods.cpp \
ok-drivers/utils/BitwiseMethods/BitwiseMethods.cpp

# ASM sources ("Assembly Language") - defines main() function
ASM_SOURCES =  \
startup_stm32f446xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
AR = $(GCC_PATH)/$(PREFIX)ar
GDB = $(GCC_PATH)/$(PREFIX)gdb
else
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
AR = $(PREFIX)ar
GDB = $(PREFIX)gdb
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################

# Specify the name of the target CPU.
CPU = -mcpu=cortex-m4

# Specify the name of the target floating point hardware/format.
FPU = -mfpu=fpv4-sp-d16

# Specify if floating point hardware should be used.
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F446xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IAPI \
-IAPI/rtos/Inc \
-IAPI/Inc \
-IAPI/cxxsupport \
-IDegree/Inc \
-IDegree/Tasks/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-Imiddleware/FreeRTOS/Source/include \
-Imiddleware/FreeRTOS/Source/CMSIS_RTOS_V2 \
-Imiddleware/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-Iok-drivers/drivers/CAP1208 \
-Iok-drivers/drivers/DAC8554 \
-Iok-drivers/drivers/SX1509 \
-Iok-drivers/drivers/IS31FL3739 \
-Iok-drivers/drivers/MPR121 \
-Iok-drivers/drivers/MCP23017 \
-Iok-drivers/drivers/TCA9548A \
-Iok-drivers/utils/Algorithms \
-Iok-drivers/utils/ArrayMethods \
-Iok-drivers/utils/BitwiseMethods \
-Iok-drivers/utils/OK_I2C \
-ISystem/Inc

CPP_INCLUDES = \

###########

# -Og                   
# -Wall	Recommended compiler warnings
# -fdata-sections
# -ffunction-sections
# -g    Generate debugging information
# -gdwarf-2
# -MMD
# -MP
# -c                       Compile and assemble, but do not link.
###########

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections 

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

ifeq ($(SERIAL_DEBUG), 1)
CFLAGS += -DSERIAL_DEBUG=1
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# C++ Flags
CPPFLAGS = $(CFLAGS) $(CPP_INCLUDES)
CPPFLAGS += \
-fno-exceptions \
-fno-rtti 

C_STANDARD = -std=gnu11
CPP_STANDARD += -std=gnu++14

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F446RETx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# helpers
#######################################
usedFlash = $$( $(SZ) $@ | sed -n 2p | awk '{print $$1}' )
usedFlashPercent = $$(( 100 * $(usedFlash) / $(FLASH_SIZE) ))
flashMessage = Flash Used: $(usedFlash)/$(FLASH_SIZE) ( $(usedFlashPercent) % )
usedRam = $$( $(SZ) $@ | sed -n 2p | awk '{ram=$$2+$$3} {print ram}' )
usedRamPercent = $$(( 100 * $(usedRam) / $(RAM_SIZE) ))
ramMessage = Ram Used: $(usedRam)/$(RAM_SIZE) ( $(usedRamPercent) % ) - (static only)

#######################################
# build the application
#######################################
# list of .c objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of .cpp objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(CPP_SOURCES:.cpp=.o))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
	@echo ""
	@echo "$(flashMessage)"
	@echo "$(ramMessage)"
	@echo ""

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
# searches for all .d files in given directory and inserts them into the .c file
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)


######################################
# OpenOCD stuff
# TODO: add config.mk file for settings like programmer, etc.
######################################
CHIPSET ?= stm32f4x
FLASH_ADDRESS ?= 0x08000000

OCD=openocd
OCD_DIR ?= /usr/local/share/openocd/scripts
PGM_DEVICE ?= interface/stlink.cfg
OCDFLAGS = -f $(PGM_DEVICE) -f target/$(CHIPSET).cfg

program:
	$(OCD) -s $(OCD_DIR) $(OCDFLAGS) \
		-c "program ./$(BUILD_DIR)/$(TARGET).elf verify reset exit"


DFU_INTERFACE_NUMBER = 0
DFU_ALT_SETTING = 0
DFU_FUSE_ADDRESS = $(FLASH_ADDRESS)

usb-upload:
	dfu-util -a $(DFU_ALT_SETTING) -s $(DFU_FUSE_ADDRESS):leave -D $(BUILD_DIR)/$(TARGET).bin
