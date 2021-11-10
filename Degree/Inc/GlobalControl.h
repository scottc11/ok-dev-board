#pragma once

#include "main.h"
#include "Degrees.h"
#include "TouchChannel.h"
#include "Callback.h"
#include "Flash.h"
#include "MCP23017.h"
#include "CAP1208.h"

namespace DEGREE {

    class GlobalControl
    {
    public:
        GlobalControl(TouchChannel *chanA_ptr,
                      TouchChannel *chanB_ptr,
                      TouchChannel *chanC_ptr,
                      TouchChannel *chanD_ptr,
                      CAP1208 *touch_ptr,
                      Degrees *degrees_ptr,
                      MCP23017 *buttons_ptr
                      ) : ioInterrupt(BUTTONS_INT), touchInterrupt(GLBL_TOUCH_INT), recLED(REC_LED, 0), freezeLED(FREEZE_LED, 0)
        {
            channels[0] = chanA_ptr;
            channels[1] = chanB_ptr;
            channels[2] = chanC_ptr;
            channels[3] = chanD_ptr;
            touchPads = touch_ptr;
            switches = degrees_ptr;
            buttons = buttons_ptr;
            ioInterrupt.fall(callback(this, &GlobalControl::handleButtonInterupt));
            touchInterrupt.fall(callback(this, &GlobalControl::handleTouchInterupt));
        };

        TouchChannel *channels[NUM_DEGREE_CHANNELS];
        CAP1208 *touchPads;
        Degrees *switches;      // degree 3-stage toggle switches io
        MCP23017 *buttons;      // io for tactile buttons
        InterruptIn ioInterrupt; // interupt pin for buttons MCP23017 io
        InterruptIn touchInterrupt; // interupt pin for touch pads
        DigitalOut recLED;
        DigitalOut freezeLED;
        
        bool buttonInterupt;        // flag for handling buttons interupt
        uint16_t currButtonsState;
        uint16_t prevButtonsState;

        bool touchDetected;
        bool gestureFlag;
        uint8_t currTouched;
        uint8_t prevTouched;

        void init();
        void poll();
        void pollButtons();
        void pollTouchPads();
        
        void handleButtonPress(int pad);
        void handleButtonRelease(int pad);
        
        void handleSwitchChange();
        void handleButtonInterupt();
        void handleTouchInterupt();
        
        void loadCalibrationDataFromFlash();

    private:
    private:
        enum PadNames : uint16_t
        { // integers correlate to 8-bit index position
            FREEZE = 0x4000,
            RECORD = 0x2000,
            RESET = 0x1000,
            CMODE = 0x0800,
            CLEAR_BEND = 0x0400,
            CLEAR_SEQ = 0x0200,
            BEND_MODE = 0x0100,
            QUANTIZE_AMOUNT = 0x0080,
            SEQ_LENGTH = 0x0020,
            PB_RANGE = 0x0040,
            SHIFT = 0x0010,
            CTRL_A = 0x0008,
            CTRL_B = 0x0004,
            CTRL_C = 0x0002,
            CTRL_D = 0x0001
        };

        enum Gestures : uint16_t
        {
            CALIBRATE_BENDER = SHIFT | BEND_MODE,                      // SHIFT + BEND_MODE
            RESET_CALIBRATION_TO_DEFAULT = SHIFT | RECORD | BEND_MODE, // SHIFT + REC + BEND_MODE
            CLEAR_SEQ_A = CLEAR_SEQ | CTRL_A,
            CLEAR_SEQ_B = CLEAR_SEQ | CTRL_B,
            CLEAR_SEQ_C = CLEAR_SEQ | CTRL_C,
            CLEAR_SEQ_D = CLEAR_SEQ | CTRL_D,
            CLEAR_BEND_SEQ_A = 0x2008,
            CLEAR_BEND_SEQ_B = 0x2004,
            CLEAR_BEND_SEQ_C = 0x2002,
            CLEAR_BEND_SEQ_D = 0x2001
        };
    };
}
