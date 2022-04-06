#pragma once

#include "main.h"
#include "okSemaphore.h"
#include "task_calibration.h"
#include "task_display.h"
#include "task_sequence_handler.h"
#include "Degrees.h"
#include "TouchChannel.h"
#include "Callback.h"
#include "Flash.h"
#include "MCP23017.h"
#include "CAP1208.h"
#include "SuperClock.h"
#include "Display.h"
#include "AnalogHandle.h"

namespace DEGREE {
    class TouchChannel; // forward declaration
    
    class GlobalControl
    {
    public:
        
        enum Mode
        {
            DEFAULT,
            CALIBRATING_1VO,
            CALIBRATING_BENDER,
            HARDWARE_TESTING
        };

        GlobalControl(
            SuperClock *clock_ptr,
            TouchChannel *chanA_ptr,
            TouchChannel *chanB_ptr,
            TouchChannel *chanC_ptr,
            TouchChannel *chanD_ptr,
            CAP1208 *touch_ptr,
            Degrees *degrees_ptr,
            MCP23017 *buttons_ptr,
            Display *display_ptr) : ioInterrupt(BUTTONS_INT, PullUp), touchInterrupt(GLBL_TOUCH_INT), recLED(REC_LED, 0), freezeLED(FREEZE_LED, 0), tempoPot(TEMPO_POT), tempoLED(TEMPO_LED), tempoGate(INT_CLOCK_OUTPUT)
        {
            mode = DEFAULT;
            clock = clock_ptr;
            channels[0] = chanA_ptr;
            channels[1] = chanB_ptr;
            channels[2] = chanC_ptr;
            channels[3] = chanD_ptr;
            touchPads = touch_ptr;
            switches = degrees_ptr;
            buttons = buttons_ptr;
            display = display_ptr;
        };

        Mode mode;
        SuperClock *clock;
        TouchChannel *channels[CHANNEL_COUNT];
        CAP1208 *touchPads;
        Degrees *switches;      // degree 3-stage toggle switches io
        MCP23017 *buttons;      // io for tactile buttons
        Display *display;
        InterruptIn ioInterrupt; // interupt pin for buttons MCP23017 io
        InterruptIn touchInterrupt; // interupt pin for touch pads
        DigitalOut recLED;
        DigitalOut freezeLED;
        AnalogHandle tempoPot;
        DigitalOut tempoLED;
        DigitalOut tempoGate;

        int selectedChannel;

        bool recordEnabled;      // global recording flag
        bool sampleVCO;          // global flag for calibration routine
        bool hardwareTesting;    // hardware testing mode

        uint16_t currButtonsState;
        uint16_t prevButtonsState;

        uint16_t currTempoPotValue;
        uint16_t prevTempoPotValue;

        bool gestureFlag;
        uint8_t currTouched;
        uint8_t prevTouched;

        void init();
        void poll();
        void pollButtons();
        void pollTouchPads();
        void pollTempoPot();

        void advanceSequencer(uint8_t pulse);
        void resetSequencer();

        void handleTempoAdjustment(uint16_t value);

        void handleButtonPress(int pad);
        void handleButtonRelease(int pad);

        void handleChannelGesture(Callback<void(int chan)> callback);
        int getTouchedChannel();

        void handleSwitchChange();
        void handleButtonInterrupt();
        void handleTouchInterrupt();

        void enableVCOCalibration(TouchChannel *channel);
        void disableVCOCalibration();

        void loadCalibrationDataFromFlash();
        void saveCalibrationDataToFlash();
        void deleteCalibrationDataFromFlash();
        void resetCalibrationDataToDefault();
        void resetCalibration1VO(int chan);
        int getCalibrationDataPosition(int data_index, int channel_index);

        void log_system_status();

        void handleHardwareTest(uint16_t pressedButtons);

    private:
    private:
        enum PadNames : uint16_t
        { // integers correlate to 8-bit index position
#if BOARD_VERSION == 38
            FREEZE = 0x4000,
            RECORD = 0x2000,
            RESET = 0x1000,
            CMODE = 0x0800,
            CLEAR_BEND = 0x0400,
            CLEAR_SEQ = 0x0200,
            BEND_MODE = 0x0100,
            QUANTIZE_SEQ = 0x0080,
            SEQ_LENGTH = 0x0020,
            PB_RANGE = 0x0040,
            SHIFT = 0x0010,
            CTRL_A = 0x0008,
            CTRL_B = 0x0004,
            CTRL_C = 0x0002,
            CTRL_D = 0x0001
#else
            FREEZE = 0x4000,
            RECORD = 0x2000,
            RESET = 0x1000,
            CMODE = 0x0080,
            CLEAR_SEQ_BEND = 0x0400,
            CLEAR_SEQ_TOUCH = 0x0200,
            BEND_MODE = 0x0040,
            QUANTIZE_SEQ = 0x0800,
            SEQ_LENGTH = 0x0100,
            PB_RANGE = 0x0020,
            SHIFT = 0x0010,
            CTRL_A = 0x0008,
            CTRL_B = 0x0004,
            CTRL_C = 0x0002,
            CTRL_D = 0x0001
#endif
        };

        enum Gestures : uint16_t
        {
            QUANTIZE_AMOUNT = SHIFT | PB_RANGE,
            CALIBRATE_BENDER = SHIFT | BEND_MODE,    // SHIFT + BEND_MODE
            RESET_CALIBRATION_DATA = SHIFT | FREEZE, // SHIFT + FREEZE
            CALIBRATE_1VO = SHIFT | CMODE,
            CLEAR_SEQ_ALL = CLEAR_SEQ_BEND | CLEAR_SEQ_TOUCH,
            ENTER_HARDWARE_TEST = SHIFT | SEQ_LENGTH | QUANTIZE_SEQ | CMODE,
            LOG_SYSTEM_STATUS = SHIFT | QUANTIZE_SEQ | SEQ_LENGTH
        };

        enum HardwareTest : uint16_t
        {
            TEST_BEND_OUT_HIGH = SHIFT,
            TEST_BEND_OUT_LOW = PB_RANGE,
            TEST_BEND_OUT_MID = BEND_MODE,
            TEST_1VO_HIGH = QUANTIZE_SEQ,
            TEST_1VO_LOW = CLEAR_SEQ_BEND,
            TEST_GATE_HIGH = CLEAR_SEQ_TOUCH,
            TEST_GATE_LOW = SEQ_LENGTH,
            EXIT_HARDWARE_TEST = ENTER_HARDWARE_TEST
        };
    };
}
