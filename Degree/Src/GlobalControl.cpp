#include "GlobalControl.h"

using namespace DEGREE;

uint32_t SETTINGS_BUFFER[SETTINGS_BUFFER_SIZE];

void GlobalControl::init() {
    suspend_sequencer_task();
    this->loadCalibrationDataFromFlash();
    this->loadChannelConfigDataFromFlash();

    display->init();
    display->clear();

    logger_log("\n");
    logger_log("\n** Global Control **");
    logger_log("\nToggle Switches: connected = ");
    logger_log(switches->io->isConnected());
    logger_log(", ISR pin (before) = ");
    logger_log(switches->ioInterupt.read());
    switches->init();
    logger_log(", ISR pin (after) = ");
    logger_log(switches->ioInterupt.read());

    logger_log("\nTactile Buttons: connected = ");
    logger_log(buttons->isConnected());
    logger_log(", ISR pin (before) = ");
    logger_log(ioInterrupt.read());
    buttons->init();
    buttons->setDirection(MCP23017_PORTA, 0xff);
    buttons->setDirection(MCP23017_PORTB, 0xff);
    buttons->setInterupt(MCP23017_PORTA, 0xff);
    buttons->setInterupt(MCP23017_PORTB, 0xff);
    buttons->setPullUp(MCP23017_PORTA, 0xff);
    buttons->setPullUp(MCP23017_PORTB, 0xff);
    buttons->setInputPolarity(MCP23017_PORTA, 0xff);
#if BOARD_VERSION == 41
    buttons->setInputPolarity(MCP23017_PORTB, 0b01111111);
#else
    buttons->setInputPolarity(MCP23017_PORTB, 0b11111111);
#endif
    logger_log(", ISR pin (after) = ");
    logger_log(ioInterrupt.read());

    logger_log("\nTouch Pads:      connected = ");
    logger_log(touchPads->isConnected());
    logger_log(", ISR pin (before) = ");
    logger_log(touchInterrupt.read());
    touchPads->init();
    logger_log(", ISR pin (after) = ");
    logger_log(touchInterrupt.read());

    // Tempo Pot ADC Noise: 1300ish w/ 100nF
    tempoPot.setFilter(0.1);
    okSemaphore *sem_ptr = tempoPot.initDenoising();
    sem_ptr->take(); // wait
    sem_ptr->give();
    tempoPot.log_noise_threshold_to_console("Tempo Pot");
    tempoPot.invertReadings();
    logger_log("\n");

    // initializing channels here might be initializing the SPI while an interrupt is getting fired by
    // the tactile buttons / switches, which may be interrupting this task and using the SPI periph before
    // it is initialized
    channels[0]->init();
    channels[1]->init();
    channels[2]->init();
    channels[3]->init();
    display->clear();

    // initialize tempo
    clock->init();
    clock->attachResetCallback(callback(this, &GlobalControl::resetSequencer));
    clock->attachPPQNCallback(callback(this, &GlobalControl::advanceSequencer)); // always do this last
    clock->attachInputCaptureCallback(callback(&calibrator, &Calibrator::captureCallback));
    clock->disableInputCaptureISR(); // pollTempoPot() will re-enable should pot be in teh right position
    currTempoPotValue = tempoPot.read_u16();
    handleTempoAdjustment(currTempoPotValue);
    prevTempoPotValue = currTempoPotValue;
    clock->start();

    switches->attachCallback(callback(this, &GlobalControl::handleSwitchChange));
    switches->enableInterrupt();
    switches->updateDegreeStates(); // not ideal, but you have to clear the interrupt after initialization
    ioInterrupt.fall(callback(this, &GlobalControl::handleButtonInterrupt));
    buttons->digitalReadAB();
    touchInterrupt.fall(callback(this, &GlobalControl::handleTouchInterrupt));
    resume_sequencer_task();
}

void GlobalControl::poll()
{
    switch (mode)
    {
    case DEFAULT:
        pollTempoPot(); // TODO: possibly move this into sequence handler on every clock tick
        break;
    case VCO_CALIBRATION:
        break;
    case CALIBRATING_BENDER:
        for (int i = 0; i < 4; i++)
        {
            uint16_t sample = channels[i]->bender->adc.read_u16();
            channels[i]->bender->adc.sampleMinMax(sample);
        }
        break;
    case HARDWARE_TESTING:
        break;
    default:
        break;
    }
}

/**
 * @brief exit out of current mode
 */
void GlobalControl::exitCurrentMode()
{
    switch (mode)
    {
    case ControlMode::DEFAULT:
        /* code */
        break;
    case ControlMode::VCO_CALIBRATION:
        actionTimer.stop();
        actionTimer.detachCallback();
        display->disableBlink();
        display->clear();
        resume_sequencer_task();
        dispatch_sequencer_event(CHAN::ALL, SEQ::DISPLAY, 0);
        // enter default mode, check if any sequences are active and running, if so, update the display
        break;
    case ControlMode::CALIBRATING_BENDER:
        /* code */
        break;

    case ControlMode::SETTING_SEQUENCE_LENGTH:
        // iterate over each channel and then keep the display LEDs on or off based on sequence containing events
        for (int chan = 0; chan < CHANNEL_COUNT; chan++)
        {
            if (!channels[chan]->sequence.containsEvents())
            {
                display->clear(chan);
            }
            channels[chan]->disableBenderOverride();
            channels[chan]->setUIMode(TouchChannel::UIMode::UI_PLAYBACK);
            display->disableBlink();
        }
        break;

    case ControlMode::SETTING_QUANTIZE_AMOUNT:
        for (int i = 0; i < CHANNEL_COUNT; i++)
        {
            channels[i]->setUIMode(TouchChannel::UIMode::UI_PLAYBACK);
        }
        break;

    case ControlMode::HARDWARE_TESTING:
        /* code */
        break;
    }
    // always revert to default mode
    mode = ControlMode::DEFAULT;
}

void GlobalControl::handleSwitchChange()
{
    dispatch_sequencer_event(CHAN::ALL, SEQ::HANDLE_DEGREE, 0);
}

void GlobalControl::handleButtonInterrupt()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t isr_id = ISR_ID_TACTILE_BUTTONS;
    xQueueSendFromISR(qhInterruptQueue, &isr_id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GlobalControl::handleTouchInterrupt() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t isr_id = ISR_ID_TOUCH_PADS;
    xQueueSendFromISR(qhInterruptQueue, &isr_id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GlobalControl::pollTempoPot()
{
    currTempoPotValue = tempoPot.read_u16();
    if (currTempoPotValue > prevTempoPotValue + 200 || currTempoPotValue < prevTempoPotValue - 200) {
        handleTempoAdjustment(currTempoPotValue);
        prevTempoPotValue = currTempoPotValue;
    }
}

void GlobalControl::handleTempoAdjustment(uint16_t value)
{
    if (value > 1000)
    {
        if (clock->externalInputMode)
        {
            clock->disableInputCaptureISR();
        }
        clock->setPulseFrequency(clock->convertADCReadToTicks(TEMPO_POT_MIN_ADC, TEMPO_POT_MAX_ADC, value));
    }
    else
    {
        // change clock source to input mode by enabling input capture ISR
        clock->enableInputCaptureISR();
    }
}

/**
 * @brief
 * | x | x | + | - | D | C | B | A |
 */
void GlobalControl::pollTouchPads() {
    prevTouched = currTouched;

#if BOARD_VERSION == 41 // touch pads do not have + - connected, and are wired differently
    currTouched = touchPads->touched() >> 4;
#else
    currTouched = touchPads->touched();
#endif
    // queue select pad
    dispatch_sequencer_event(CHAN::ALL, SEQ::HANDLE_SELECT_PAD, 0);
    if (currTouched == 0x00) {
        gestureFlag = false;
    } else {
        gestureFlag = true;
    }
}

/**
 * Poll IO and see if any buttons have been either pressed or released
*/
void GlobalControl::pollButtons()
{
    currButtonsState = buttons->digitalReadAB();
    if (currButtonsState != prevButtonsState)
    {
        for (int i = 0; i < 16; i++)
        {
            // if state went HIGH and was LOW before
            if (bitwise_read_bit(currButtonsState, i) && !bitwise_read_bit(prevButtonsState, i))
            {
                if (mode == ControlMode::HARDWARE_TESTING)
                {
                    this->handleHardwareTest(currButtonsState);
                }
                else
                {
                    this->handleButtonPress(currButtonsState);
                }
            }
            // if state went LOW and was HIGH before
            if (!bitwise_read_bit(currButtonsState, i) && bitwise_read_bit(prevButtonsState, i))
            {
                if (mode != ControlMode::HARDWARE_TESTING)
                {
                    this->handleButtonRelease(prevButtonsState);
                }
            }
        }
    }

    // reset polling
    prevButtonsState = currButtonsState;
}

/**
 * Handle Button Press
*/
void GlobalControl::handleButtonPress(int pad)
{
    // let any butten press break out of currently running "mode"
    if (actionExitFlag == ACTION_EXIT_STAGE_1)
    {
        actionExitFlag = ACTION_EXIT_STAGE_2;
        return;
    }
    switch (pad)
    {
    case CMODE:
        if (recordEnabled == true) break;

        for (int i = 0; i < 4; i++)
        {
            if (touchPads->padIsTouched(i, currTouched))
            {
                dispatch_sequencer_event(CHAN(i), SEQ::TOGGLE_MODE, 0);
            }
        }
        break;
    case SHIFT:
        break;
    case FREEZE:
        if (recordEnabled == true) break;
        freezeLED.write(HIGH);
        this->handleFreeze(true);
        break;

    case RESET:
        if (gestureFlag)
        {
            for (int i = 0; i < CHANNEL_COUNT; i++) {
                if (touchPads->padIsTouched(i, currTouched))
                    dispatch_sequencer_event(CHAN(i), SEQ::RESET, 0);
            }
        } else {
            dispatch_sequencer_event(CHAN::ALL, SEQ::RESET, 0);
        }
        break;

    case QUANTIZE_SEQ:
        dispatch_sequencer_event(CHAN::ALL, SEQ::QUANTIZE, 0);
        break;

    case Gestures::CALIBRATE_BENDER:
        if (recordEnabled == true) break;
        if (this->mode == CALIBRATING_BENDER)
        {
            this->saveCalibrationDataToFlash();
            display->disableBlink();
            this->mode = DEFAULT;
            logger_log("\nEXIT Bender Calibration");
            resume_sequencer_task();
        }
        else
        {
            suspend_sequencer_task();
            logger_log("\nENTER Bender Calibration");
            this->mode = CALIBRATING_BENDER;
            display->enableBlink();
            display->benderCalibration();
            for (int i = 0; i < 4; i++)
            {
                channels[i]->bender->adc.resetMinMax();
            }
        }
        break;

    case Gestures::SETTINGS_RESET:
        if (recordEnabled == true) break;

        if (gestureFlag)
        {
            for (int i = 0; i < CHANNEL_COUNT; i++)
            {
                if (touchPads->padIsTouched(i, currTouched))
                {
                    this->resetCalibration1VO(i);
                }
            }
            this->saveCalibrationDataToFlash(); // change this to just save calibration data
        } else {
            this->deleteChannelConfigDataFromFlash();
        }
        break;

    case Gestures::SETTINGS_SAVE:
        if (recordEnabled == true) break;

        this->saveChannelConfigDataToFlash();
        break;

    case Gestures::CALIBRATE_1VO:
        if (recordEnabled == true) break;
        this->enableVCOCalibration();
        break;
    
    case Gestures::ENTER_HARDWARE_TEST:
        mode = ControlMode::HARDWARE_TESTING;
        break;

    case Gestures::LOG_SYSTEM_STATUS:
        log_system_status();
        break;

    case BEND_MODE:
        // iterate over currTouched and setChannelBenderMode if touched
        for (int i = 0; i < CHANNEL_COUNT; i++)
        {
            if (touchPads->padIsTouched(i, currTouched))
            {
                channels[i]->setBenderMode();
            }
        }
        break;

    case CLEAR_SEQ_TOUCH:
        break;

    case PB_RANGE:
        if (recordEnabled == true) break;
        
        for (int i = 0; i < CHANNEL_COUNT; i++)
        {
            channels[i]->setUIMode(TouchChannel::UIMode::UI_PITCH_BEND_RANGE);
        }
        break;

    case QUANTIZE_AMOUNT:
        if (recordEnabled == true) break;
        actionExitFlag = ACTION_EXIT_STAGE_1;
        mode = ControlMode::SETTING_QUANTIZE_AMOUNT;
        for (int i = 0; i < CHANNEL_COUNT; i++)
        {
            channels[i]->setUIMode(TouchChannel::UIMode::UI_QUANTIZE_AMOUNT);
        }
        break;

    case SEQ_LENGTH:
        if (recordEnabled == true) break;
        actionExitFlag = ACTION_EXIT_STAGE_1;
        mode = ControlMode::SETTING_SEQUENCE_LENGTH;
        for (int chan = 0; chan < CHANNEL_COUNT; chan++)
        {
            channels[chan]->setUIMode(TouchChannel::UIMode::UI_SEQUENCE_LENGTH);
            channels[chan]->enableBenderOverride();
            display->enableBlink();
            channels[chan]->drawSequenceToDisplay(true);
        }
        break;

    case RECORD:
        if (!recordEnabled)
        {
            recLED.write(1);
            dispatch_sequencer_event(CHAN::ALL, SEQ::RECORD_ENABLE, 0);
            recordEnabled = true;
        }
        else
        {
            recLED.write(0);
            dispatch_sequencer_event(CHAN::ALL, SEQ::RECORD_DISABLE, 0);
            recordEnabled = false;
        }
        break;
    }
}

/**
 * Handle Button Release
*/
void GlobalControl::handleButtonRelease(int pad)
{
    if (actionExitFlag == ACTION_EXIT_STAGE_2)
    {
        exitCurrentMode();
        actionExitFlag = ACTION_EXIT_CLEAR;
        return;
    }
    switch (pad)
    {
    case FREEZE:
        if (recordEnabled == true)
            break;
        freezeLED.write(LOW);
        handleFreeze(false);
        break;
    case RESET:
        break;
    case PB_RANGE:
        if (recordEnabled == true)
            break;
        for (int i = 0; i < CHANNEL_COUNT; i++)
        {
            channels[i]->setUIMode(TouchChannel::UIMode::UI_PLAYBACK);
        }
        break;
        
    case CLEAR_SEQ_TOUCH:
        if (!gestureFlag)
        {
            dispatch_sequencer_event(CHAN::ALL, SEQ::CLEAR_TOUCH, 0);
            if (!recordEnabled)
                dispatch_sequencer_event(CHAN::ALL, SEQ::RECORD_DISABLE, 0); // why do you even have to do this?
        }
        else // clear only curr touched channels sequences
        {
            for (int i = 0; i < CHANNEL_COUNT; i++)
            {
                if (touchPads->padIsTouched(i, currTouched))
                {
                    dispatch_sequencer_event((CHAN)i, SEQ::CLEAR_TOUCH, 0);
                    if (!recordEnabled)
                        dispatch_sequencer_event((CHAN)i, SEQ::RECORD_DISABLE, 0);
                }
            }
            gestureFlag = false;
        }
        break;

    case CLEAR_SEQ_BEND:
        if (!gestureFlag) {
            dispatch_sequencer_event(CHAN::ALL, SEQ::CLEAR_BEND, 0);
            if (!recordEnabled)
                dispatch_sequencer_event(CHAN::ALL, SEQ::RECORD_DISABLE, 0); // why do you even have to do this?
        } else {
            for (int i = 0; i < CHANNEL_COUNT; i++)
            {
                if (touchPads->padIsTouched(i, currTouched))
                {
                    dispatch_sequencer_event((CHAN)i, SEQ::CLEAR_BEND, 0);
                    if (!recordEnabled)
                        dispatch_sequencer_event((CHAN)i, SEQ::RECORD_DISABLE, 0);
                }
            }
            gestureFlag = false;
        }
        break;
    case SEQ_LENGTH:
        break;
    case RECORD:
        break;
    }
}

bool validateFirmwareVersionMatch(uint32_t *arr) {
    char *string = FIRMWARE_VERSION;
    int string_len = strlen(string);
    
    if (string_len > FLASH_FIRMWARE_VERSION_SIZE) {
        string_len = FLASH_FIRMWARE_VERSION_SIZE;
    }
    
    // if any of the values in the buffer don't match the respective values in the firmware string array
    // then there is no version match, and the config settings need to be reset / cleared / not used
    bool versionMatch;
    for (int i = 0; i < string_len; i++)
    {
        if (arr[i] == (uint32_t)string[i])
        {
            versionMatch = true;
        }
        else
        {
            versionMatch = false;
            break;
        }
    }
    return versionMatch;
}

/**
 * NOTE: Careful with the creation of this buffer, as it is quite large and may push the memory past its limits
*/ 
void GlobalControl::loadCalibrationDataFromFlash()
{
    Flash flash;
    flash.read(FLASH_FIRMWARE_VERSION_ADDR, (uint32_t *)SETTINGS_BUFFER, FLASH_FIRMWARE_VERSION_SIZE);

    bool configDataEmpty = flash.validate(SETTINGS_BUFFER, 4);
    
    bool isVersionMatch = validateFirmwareVersionMatch(SETTINGS_BUFFER);

    // if no config data or firmware version mismatch, load default configuration
    if (configDataEmpty || !isVersionMatch)
    {
        // load default 1VO values
        logger_log("\nChannel Settings Source: DEFAULT");
        for (int chan = 0; chan < CHANNEL_COUNT; chan++)
        {
            channels[chan]->output.resetVoltageMap();
        }
    }
    else // load the data from flash
    {
        logger_log("\nChannel Settings Source: FLASH");
        // load calibration data
        for (int chan = 0; chan < CHANNEL_COUNT; chan++)
        {
            uint32_t address_offset = FLASH_CALIBRATION_BLOCK_SIZE * chan;
            flash.read(FLASH_1VO_CALIBRATION_ADDR + address_offset, SETTINGS_BUFFER, DAC_1VO_ARR_SIZE);
            for (int i = 0; i < DAC_1VO_ARR_SIZE; i++)
            {
                channels[chan]->output.dacVoltageMap[i] = (uint16_t)SETTINGS_BUFFER[i];
            }
            flash.read(FLASH_BENDER_CALIBRATION_ADDR, SETTINGS_BUFFER, 2);
            channels[chan]->bender->setMinBend((uint16_t)SETTINGS_BUFFER[0]);
            channels[chan]->bender->setMaxBend((uint16_t)SETTINGS_BUFFER[1]);
        }
    }
}

void GlobalControl::loadChannelConfigDataFromFlash()
{
    Flash flash;

    flash.read(FLASH_CONFIG_ADDR, (uint32_t *)SETTINGS_BUFFER, 8);
    bool configDataEmpty = flash.validate(SETTINGS_BUFFER, 8); // if empty the first 8 words should all equal 0xFFFFFFFF

    flash.read(FLASH_FIRMWARE_VERSION_ADDR, (uint32_t *)SETTINGS_BUFFER, FLASH_FIRMWARE_VERSION_SIZE);
    bool isVersionMatch = validateFirmwareVersionMatch(SETTINGS_BUFFER);
    
    // load channel config and sequence data
    if (!configDataEmpty && isVersionMatch)
    {
        for (int chan = 0; chan < CHANNEL_COUNT; chan++)
        {
            uint32_t address_offset = FLASH_CHANNEL_BLOCK_SIZE * chan;

            // load channel config
            uint32_t channel_config[8];
            flash.read(FLASH_CHANNEL_CONFIG_ADDR + address_offset, channel_config, 8);
            channels[chan]->loadConfigData(channel_config);

            // load sequence data
            uint32_t sequence_config[4];
            flash.read(FLASH_SEQUENCE_CONFIG_ADDR + address_offset, sequence_config, 4);
            channels[chan]->sequence.loadSequenceConfigData(sequence_config);

            if (channels[chan]->sequence.containsEvents())
            {
                // read a single word (4 bytes) into array, then decode it and store in seqeuence event struct
                int addr = 0;
                for (int i = 0; i < channels[chan]->sequence.lengthPPQN; i++)
                {
                    uint32_t event_data = flash.read_word((void *)(FLASH_SEQUENCE_DATA_ADDR + addr + address_offset));
                    channels[chan]->sequence.decodeEventData(i, event_data);
                    addr += 4;
                }
            }
        }
    }
}

/**
 * @brief Save all 4 channels calibration data to flash
 * 
 * NOTE: every time we calibrate a channel, all 4 channels calibration data needs to be re-saved to flash
 * because we have to delete/clear an entire sector of data first.
*/
void GlobalControl::saveCalibrationDataToFlash()
{
    this->display->fill(PWM::PWM_MID, true);

    Flash flash;
    flash.erase(FLASH_CALIBRATION_ADDR);

    // Step 1: copy firmware version to flash
    char *string = FIRMWARE_VERSION;
    int string_len = strlen(string);
    uint32_t firmware_version[string_len];
    for (int i = 0; i < string_len; i++)
    {
        firmware_version[i] = (uint32_t)string[i];
    }
    flash.write(FLASH_FIRMWARE_VERSION_ADDR, firmware_version, string_len);

    // Step 2: copy calibration data into flash
    int buffer_position = 0;
    for (int chan = 0; chan < CHANNEL_COUNT; chan++) // channel iterrator
    {
        uint32_t address_offset = FLASH_CALIBRATION_BLOCK_SIZE * chan;
        
        // 1VO calibration data
        for (int i = 0; i < DAC_1VO_ARR_SIZE; i++)
        {
            SETTINGS_BUFFER[i] = channels[chan]->output.dacVoltageMap[i];
        }
        flash.write(FLASH_1VO_CALIBRATION_ADDR + address_offset, SETTINGS_BUFFER, DAC_1VO_ARR_SIZE);

        // max and min Bender calibration data
        SETTINGS_BUFFER[0] = channels[chan]->bender->adc.getInputMin();
        SETTINGS_BUFFER[1] = channels[chan]->bender->adc.getInputMax();
        flash.write(FLASH_BENDER_CALIBRATION_ADDR + address_offset, SETTINGS_BUFFER, 2);
    }

    // flash the grid of leds on and off for a sec then exit
    this->display->flash(3, 300);
    this->display->clear();
    logger_log("\nSaved Calibration Data to Flash");
}

/**
 * @brief Save channel sequence data and configuration data to flash
 *
 */
void GlobalControl::saveChannelConfigDataToFlash()
{
    this->display->fill(PWM::PWM_MID, true);

    Flash flash;
    flash.erase(FLASH_CONFIG_ADDR);
    for (int chan = 0; chan < CHANNEL_COUNT; chan++)
    {
        uint32_t address_offset = FLASH_CHANNEL_BLOCK_SIZE * chan;

        // store channel config
        uint32_t channel_config[8];
        channels[chan]->copyConfigData(channel_config);
        flash.write(FLASH_CHANNEL_CONFIG_ADDR + address_offset, channel_config, 8);

        // store sequence config
        uint32_t sequence_config[8];
        channels[chan]->sequence.storeSequenceConfigData(sequence_config);
        flash.write(FLASH_SEQUENCE_CONFIG_ADDR + address_offset, sequence_config, 8);

        // if a sequence exists, store that in flash as well
        if (channels[chan]->sequence.containsEvents())
        {
            // Max sequence size per channel = (32 bits per event / 4 bytes) * MAX_SEQ_LENGTH_PPQN = 12,288 bytes
            uint16_t flashWrites = channels[chan]->sequence.length;
            uint32_t sequence_data[PPQN];
            // ensure no RAM overflow by writing to flash in smaller chunks
            for (int x = 0; x < flashWrites; x++)
            {
                for (int i = 0; i < PPQN; i++)
                {
                    int pos = (x * PPQN) + i;
                    sequence_data[i] = channels[chan]->sequence.encodeEventData(pos); // encode sequence data to be stored in 32-bit chunks
                }
                uint32_t address = (FLASH_SEQUENCE_DATA_ADDR + (x * 96 * 4)) + address_offset;
                flash.write(address, sequence_data, PPQN);
            }
        }
    }

    // flash the grid of leds on and off for a sec then exit
    this->display->flash(3, 300);
    this->display->clear();
    logger_log("\nSaved Calibration Data to Flash");
}

void GlobalControl::deleteCalibrationDataFromFlash()
{
    display->setScene(SCENE::SETTINGS);
    display->resetScene();
    display->enableBlink();
    display->fill(127, true);

    Flash flash;
    flash.erase(FLASH_CALIBRATION_ADDR);

    for (int i = 0; i < DISPLAY_COLUMN_COUNT; i++)
    {
        int led = DISPLAY_COLUMN_COUNT - 1 - i; // go backwards
        display->setColumn(led, 30, true);
        vTaskDelay(50);
    }
    display->setScene(SCENE::SEQUENCER);
    display->redrawScene();
}

/**
 * @brief delete / clear all channel config data in flash
 * 
 */
void GlobalControl::deleteChannelConfigDataFromFlash()
{
    display->setScene(SCENE::SETTINGS);
    display->resetScene();
    display->enableBlink();
    display->fill(127, true);

    Flash flash;
    flash.erase(FLASH_CONFIG_ADDR);

    for (int i = 0; i < DISPLAY_COLUMN_COUNT; i++)
    {
        int led = DISPLAY_COLUMN_COUNT - 1 - i; // go backwards
        display->setColumn(led, 30, true);
        vTaskDelay(50);
    }
    display->setScene(SCENE::SEQUENCER);
    display->redrawScene();
}

void GlobalControl::resetCalibrationDataToDefault()
{
    this->deleteCalibrationDataFromFlash();
    this->loadCalibrationDataFromFlash();
}

/**
 * @brief reset a channels 1vo calibration data
 * 
 * @param chan index
 */
void GlobalControl::resetCalibration1VO(int chan)
{
    // basically just reset a channels voltage map to default and then save as usual
    channels[chan]->output.resetVoltageMap();
    // this->saveCalibrationDataToFlash(); // change this to just save calibration data
}

/**
 * @brief Called within ISR, advances all channels sequence by 1, and handles global clock output
 * 
 * @param pulse 
 */
void GlobalControl::advanceSequencer(uint8_t pulse)
{
    // if (pulse % 16 == 0)
    // {
    //     display_dispatch_isr(DISPLAY_ACTION::PULSE_DISPLAY, CHAN::ALL, 0);
    // }

    if (pulse == 0) {
        tempoLED.write(HIGH);
        tempoGate.write(HIGH);
    } else if (pulse == 4) {
        tempoLED.write(LOW);
        tempoGate.write(LOW);
    }

    dispatch_sequencer_event_ISR(CHAN::ALL, SEQ::ADVANCE, pulse);
}

/**
 * @brief reset all sequencers PPQN position to 0
 * 
 * The issue might be that the sequence is not always running too slow, but running to fast. In other words, the
 * pulse count overtakes the external clocks rising edge. To avoid this scenario, you need to halt further execution of the sequence should it reach PPQN - 1 
 * prior to the ext clock rising edge. Thing is, the sequence would first need to know that it is being externally clocked...
 * 
 * Currently, your clock division is very off. The higher the ext clock signal is, you lose an increasing amount of pulses.
 * Ex. @ 120bpm sequence will reset on pulse 84 (ie. missing 12 PPQNs)
 * Ex. @ 132bpm sequence missing 20 PPQNs
 */
void GlobalControl::resetSequencer(uint8_t pulse)
{
    dispatch_sequencer_event_ISR(CHAN::ALL, SEQ::CORRECT, 0);
}

void GlobalControl::handleFreeze(bool freeze) {
    if (this->gestureFlag)
    {
        for (int i = 0; i < CHANNEL_COUNT; i++)
        {
            if (touchPads->padIsTouched(i, currTouched))
                dispatch_sequencer_event(CHAN(i), SEQ::FREEZE, freeze);
        }
    }
    else
    {
        dispatch_sequencer_event(CHAN::ALL, SEQ::FREEZE, freeze);
    }
}

void GlobalControl::enableVCOCalibration() {
    int touchedChannel = getTouchedChannel();
    mode = ControlMode::VCO_CALIBRATION;
    suspend_sequencer_task();
    display->enableBlink();
    display->fill(30, true);
    for (int i = 0; i < 16; i++)
    {
        display->setSpiralLED(touchedChannel, i, 255, false);
    }
    ctrl_dispatch(CTRL_ACTION::ENTER_1VO_CALIBRATION, touchedChannel, 0);
}

void GlobalControl::disableVCOCalibration() {
    this->mode = ControlMode::DEFAULT;
}

/**
 * @brief auto-reload software timer callback function which steps through the currently selected channels 16 display LEDs
 * and makes them brighter. Once function increments its counter to 16, enter that channel into VCO calibration
 */
void GlobalControl::pressHold() {
    if (gestureFlag)
    {
        int touchedChannel = getTouchedChannel();
        display->setSpiralLED(touchedChannel, actionCounter, 255, false);
        actionCounter++;
        if (actionCounter > actionCounterLimit) {
            actionTimer.stop();
            selectedChannel = touchedChannel; // this is kinda meh
            ctrl_dispatch(CTRL_ACTION::ENTER_1VO_CALIBRATION, touchedChannel, 0);
        }
    } else {
        // reset
        if (actionCounter != 0)
        {
            display->fill(30, true);
        }
        actionCounter = 0;
    }
}

void GlobalControl::handleChannelGesture(Callback<void(int chan)> callback)
{
    for (int i = 0; i < CHANNEL_COUNT; i++)
    {
        if (touchPads->padIsTouched(i, currTouched))
        {
            callback(i);
            break;
        }
    }
}

/**
 * @brief return the currently touched channel
 * 
 * @return int 
 */
int GlobalControl::getTouchedChannel() {
    int channel = 0;
    for (int i = 0; i < CHANNEL_COUNT; i++)
    {
        if (touchPads->padIsTouched(i, currTouched))
        {
            channel = i;
            break;
        }
    }
    return channel;
}

void GlobalControl::log_system_status()
{
    logger_log("\nToggle Switches ISR = ");
    logger_log(switches->ioInterupt.read());

    logger_log("\nTactile Buttons ISR pin = ");
    logger_log(ioInterrupt.read());

    logger_log("\nTouch ISR pin = ");
    logger_log(touchInterrupt.read());
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        channels[i]->logPeripherals();
        channels[i]->output.logVoltageMap();
    }
}

void GlobalControl::handleHardwareTest(uint16_t pressedButtons)
{
    switch (pressedButtons)
    {
    case HardwareTest::TEST_BEND_OUT_HIGH:
        for (int i = 0; i < CHANNEL_COUNT; i++)
            channels[i]->bender->updateDAC(BIT_MAX_16, true);
        break;
    case HardwareTest::TEST_BEND_OUT_LOW:
        for (int i = 0; i < CHANNEL_COUNT; i++)
            channels[i]->bender->updateDAC(0, true);
        break;
    case HardwareTest::TEST_BEND_OUT_MID:
        for (int i = 0; i < CHANNEL_COUNT; i++)
            channels[i]->bender->updateDAC(BENDER_DAC_ZERO, true);
        break;
    case HardwareTest::TEST_1VO_HIGH:
        for (int i = 0; i < CHANNEL_COUNT; i++)
            channels[i]->output.dac->write(channels[i]->output.dacChannel, BIT_MAX_16);
        break;
    case HardwareTest::TEST_1VO_LOW:
        for (int i = 0; i < CHANNEL_COUNT; i++)
            channels[i]->output.dac->write(channels[i]->output.dacChannel, 0);
        break;
    case HardwareTest::TEST_GATE_HIGH:
        for (int i = 0; i < CHANNEL_COUNT; i++)
            channels[i]->setGate(true);
        break;
    case HardwareTest::TEST_GATE_LOW:
        for (int i = 0; i < CHANNEL_COUNT; i++)
            channels[i]->setGate(false);
        break;
    case HardwareTest::EXIT_HARDWARE_TEST:
        mode = ControlMode::DEFAULT;
        break;
    default:
        break;
    }
}