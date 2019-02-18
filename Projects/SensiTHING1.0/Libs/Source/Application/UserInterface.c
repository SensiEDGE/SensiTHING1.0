/***************************************************************************//**
 * @file   UserInterface.c
 * @brief  Provides some logic and button and leds processing.
 *
 *******************************************************************************
 * COPYRIGHT(c) 2019 SensiEDGE
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of SensiEDGE nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "hal-config.h"

#include "SeGpio.h"
#include "Battery.h"
#include "Buzzer.h"
#include "Environmental.h"
#include "Smoke.h"
#include "Accelerometer.h"
#include "GpioAdcDac.h"
#include "Standby.h"
#include "ApplicationTimer.h"
#include "UserInterface.h"
#include "TimeUtils.h"



/* Local objects. */

/* Button structure. */
typedef struct {
    SeGpio_pin_t    m_pin;
    uint32_t        m_changeTime;
    uint32_t        m_pressedTime;
    uint32_t        m_releasedTime;
    bool            m_state;
    bool            m_longPressed;
} Button_t;

static Button_t Button;

/* Button settings. */
static const uint32_t kButtonPressTimeShortMs = 300;
static const uint32_t kButtonPressTimeLongMs = 3000;
static const uint32_t kButtonDebounceFilterMs = 20;

/* Leds settings. */
static const uint32_t kLedBlinkTimeOffMs = 950;
static const uint32_t kLedBlinkTimeOnMs = 50;


/* A flag indicating the presence of a smartphone/tablet/PC connection. */
static bool IsConnected = false;

/* A flag indicating the phase of the LED's light when it blinking. */
static bool LedsOnPhase = false;


/* Private function prototypes */
static void UserInterface_buttonShort(void);
static void UserInterface_buttonLong(void);
static void UserInterface_extIntCallback(uint8_t pin);
static bool UserInterface_debounceFilter(Button_t *button);
static void UserInterface_ledsControl(void);


/* Public functions. */

/**
 * @brief      Verify the conditions for start the application.
 * @retval     None.
 */
void UserInterface_verifyStart(void)
{
    if (Standby_isStartFromStandby()) {
        TRACE_INFO_WP("\nStart from standby \n");

        // Init button pin
        SeGpio_pin_t Button = {
            .m_pin = BSP_BUTTON_PIN,
            .m_port = BSP_BUTTON_PORT,
        };

        SeGpio_configPin(&Button, SE_GPIO_MODE_INPUT_PULL_FILTER, true);

        uint32_t l_time = TimeUtils_getMs();

        while (TimeUtils_getMs() - l_time < kButtonPressTimeLongMs) {
            // If button released put system to standby mode
            if (SeGpio_getInPin(&Button)) {
                TimeUtils_delay(100);
                TRACE_INFO_WP("Put to standby \n");

                Standby_enter();
            }
        }

    }

}

/**
 * @brief      Initialize user interface module.
 * @retval     None.
 */
void UserInterface_init(void)
{
    // Button configuration
    Button.m_pin.m_pin = BSP_BUTTON_PIN;
    Button.m_pin.m_port = BSP_BUTTON_PORT;

    SeGpio_configExtIntPin(&Button.m_pin, SE_GPIO_MODE_INPUT_PULL_FILTER, true, SE_GPIO_INT_MODE_RISING_FALLING, UserInterface_extIntCallback);
    SeGpio_enableExtInterrupt(&Button.m_pin, true);

    // Turn ON red led at startup
    GpioAdcDac_setLedRed(true);
    // Start timer for leds
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(kLedBlinkTimeOnMs), EVT_TIMER_UI_LEDS, true);
    LedsOnPhase = true;

    Buzzer_on();
    TimeUtils_delay(100);
    Buzzer_off();

}

/**
 * @brief      Function that is called when the software timer is triggered.
 * @param[in]  uint16_t timId - Timer ID (@ref ApplicationTimer_t).
 * @retval     None.
 */
void UserInterface_timHandler(uint16_t timId)
{
    /* Check which software timer handle is in question */
    switch (timId) {
        case EVT_TIMER_UI_BUTTON_SHORT:
            UserInterface_buttonShort();
            break;
        case EVT_TIMER_UI_BUTTON_LONG:
            UserInterface_buttonLong();
            break;
        case EVT_TIMER_UI_LEDS:
            UserInterface_ledsControl();
            break;
        default:
            break;
    }
}

/**
 * @brief      Update connection state.
 * @param[in]  bool state - New state of client connection.
 * @retval     None.
 */
void UserInterface_updConnectionState(bool state)
{
    IsConnected = state;

    // Reset leds
    GpioAdcDac_setLedRed(false);
    GpioAdcDac_setLedGreen(false);

}


/* Private functions */

/**
 * @brief      Processing a short button press.
 * @retval     None.
 */
static void UserInterface_buttonShort(void)
{

    if(UserInterface_debounceFilter(&Button)) {
        // Actions when state is changed
        if (Button.m_state) {
            // Start checking long pressed event
            Button.m_longPressed = true;
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(kButtonPressTimeLongMs), EVT_TIMER_UI_BUTTON_LONG, true);
        } else {
            if (Button.m_pressedTime < kButtonPressTimeShortMs) {
                TRACE_INFO_WP("Click\n");
            }
            // Stop checking long pressed event
            if(Button.m_longPressed) {
                Button.m_longPressed = false;
                gecko_cmd_hardware_set_soft_timer(TIMER_STOP, EVT_TIMER_UI_BUTTON_LONG, true);
            }
        }
    }

}

/**
 * @brief      Processing a long button press.
 * @retval     None.
 */
static void UserInterface_buttonLong(void)
{
    // Action when button long pressed
    if(Button.m_longPressed && Button.m_state) {

        if((TimeUtils_getMs() - Button.m_changeTime) >= kButtonPressTimeLongMs) {

            /* Stop all modules */
            Battery_off();
            Buzzer_off();
            Environmental_off();
            Smoke_off();
            Accelerometer_off();
            GpioAdcDac_triState();
            GpioAdcDac_off();

            Buzzer_on();
            TimeUtils_delay(100);
            Buzzer_off();

            while(!SeGpio_getInPin(&Button.m_pin)) {

            }

            TRACE_INFO_WP("Standby\n");

            TimeUtils_delay(10);

            Standby_enter();

        }
    }
}

/**
 * @brief      Handler for external interrupt.
 * @param[in]  uint8_t pin - The pin index the callback function is invoked for.
 * @retval     None.
 */
static void UserInterface_extIntCallback(uint8_t pin)
{
    if(pin == Button.m_pin.m_pin)  {
        gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(kButtonDebounceFilterMs), EVT_TIMER_UI_BUTTON_SHORT, true);
    }
}

/**
 * @brief      Handler for external interrupt.
 * @param[in]  Button_t *button - Pointer to button structure.
 * @retval     bool - Button state. 'true' - if button pressed, 'false' - otherwise.
 */
static bool UserInterface_debounceFilter(Button_t *button)
{
    // If button pressed
    if (!SeGpio_getInPin(&button->m_pin) && !button->m_state) {
        button->m_state = true;
        button->m_releasedTime = TimeUtils_getMs() - button->m_changeTime;
        button->m_changeTime = TimeUtils_getMs();
        return true;
    }
    // If button released
    else if (SeGpio_getInPin(&button->m_pin) && button->m_state) {
        button->m_state = false;
        button->m_pressedTime = TimeUtils_getMs() - button->m_changeTime;
        button->m_changeTime = TimeUtils_getMs();
        return true;
    }

    return false;
}

/**
 * @brief      Leds control.
 * @retval     None.
 */
static void UserInterface_ledsControl(void)
{

    if(IsConnected) {
        GpioAdcDac_setLedGreen(!LedsOnPhase);
    } else {
        GpioAdcDac_setLedRed(!LedsOnPhase);
    }

    struct gecko_msg_hardware_set_soft_timer_rsp_t* resp;

    // Set next period
    uint32_t l_time = LedsOnPhase ? kLedBlinkTimeOffMs : kLedBlinkTimeOnMs;
    resp = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(l_time), EVT_TIMER_UI_LEDS, false);
    LedsOnPhase = !LedsOnPhase;

    if(resp->result) {
        TRACE_INFO_WP("UserInterface_ledsControl: error 0x%04X \n",resp->result);
    }
}

