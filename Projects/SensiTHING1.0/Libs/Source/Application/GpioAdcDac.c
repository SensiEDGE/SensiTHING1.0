/***************************************************************************//**
 * @file   GpioAdcDac.c
 * @brief  Functions and data related to GPIO/ADC/DAC module.
 *         The module configures the chip outputs as:
 *
 *         I/O2 - Logic output
 *         I/O4 - Logic input
 *         I/O6 - Analog-to-digital converter (ADC) input
 *         I/O7 - Digital-to-analog converter (DAC) output
 *         I/O6 - System red led
 *         I/O7 - System green led
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

#include "ApplicationTimer.h"
#include "AD5592R.h"
#include "GpioAdcDac.h"
#include "Buzzer.h"

/* Local objects */
static AD5592R_t Ad5592r;
static AD5592R_t *Driver = &Ad5592r;
static SeSpi_t *hSpi;

/* Custom SPI mode for chip. */
static SeSpi_mode_t SpiModeAD5592R;
static SeSpi_mode_t SpiModeDefault;

/* Pins used by the chip. */
static SeGpio_pin_t ChipSelectPin;
static SeGpio_pin_t ResetPin;

static bool RedState = false;

/* Connection ID. */
static uint8_t ClientConnection = 0;

/* Data sending period. */
static const uint32_t kSendPeriodMs = 500;

/* Pin configuration. */
typedef enum {
    OUT0_PIN = 0,
    OUT1_PIN,
    IN0_PIN,
    IN1_PIN,
    ADC_PIN,
    DAC_PIN,
	LED_GREEN_PIN,
    LED_RED_PIN,
	//LED_GREEN_PIN,

} GpioAdcDac_pins_t;


/* Private function prototypes */
static void GpioAdcDac_sendAdcData(void);
static void GpioAdcDac_sendLedState(void);
static void GpioAdcDac_setSpiMode(SeSpi_mode_t *mode);
static void GpioAdcDac_pinsDefaultConfig(void);
static void GpioAdcDac_start(void);
static void GpioAdcDac_stop(void);


/* Public functions */

/**
 * @brief      Initialize GPIO/ADC/DAC module.
 * @param[in]  SeSpi_t *hspi - Pointer to the SPI with which AD5592R works.
 * @retval     None
 */
void GpioAdcDac_init(SeSpi_t *hspi)
{
    hSpi = hspi;

    // Config cs pin.
    ChipSelectPin.m_pin = BSP_AD5592R_CS_PIN;
    ChipSelectPin.m_port = BSP_AD5592R_CS_PORT;
    SeGpio_configPin(&ChipSelectPin, SE_GPIO_MODE_PUSH_PULL, true);

    // Config reset pin.
    ResetPin.m_pin = BSP_AD5592R_RESET_PIN;
    ResetPin.m_port = BSP_AD5592R_RESET_PORT;
    SeGpio_configPin(&ResetPin, SE_GPIO_MODE_PUSH_PULL, true);

    // Hardware reset
    SeGpio_setOutPin(&ResetPin, false);
    WAIT(1);
    SeGpio_setOutPin(&ResetPin, true);
    WAIT(1);

    // Save default SPI mode
    SeSpi_getMode(hSpi, &SpiModeDefault);

    // Init custom SPI mode for AD5592R.
    SpiModeAD5592R.m_baudrate = 1000000;
    SpiModeAD5592R.m_clockMode = spidrvClockMode1;
    SpiModeAD5592R.m_frameLength = 8;
    SpiModeAD5592R.m_bitOrder = spidrvBitOrderMsbFirst;

    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    if(!AD5592R_init(Driver, &ChipSelectPin, hSpi)) {
        TRACE_INFO_WP("AD5592R_init FAIL\n");
        // Restore default SPI mode
        GpioAdcDac_setSpiMode(&SpiModeDefault);
        return;
    }

    AD5592R_adcEnableBuffer(Driver, true);
    AD5592R_setIntReference(Driver, true);

    GpioAdcDac_pinsDefaultConfig();

    AD5592R_powerDown(Driver, true);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);

    TRACE_INFO_WP("AD5592R_init OK\n");
}

/**
 * @brief      Function that is called when the characteristic status is changed.
 * @param[in]  uint8_t connection - Connection ID.
 * @param[in]  uint16_t attribute - Attribute value.
 * @param[in]  uint16_t flags - New value of Client Characteristic Config.
 * @retval     None.
 */
void GpioAdcDac_charStatusChange(uint8_t connection, uint16_t attribute, uint16_t flags)
{

    if(gattdb_led_state == attribute) {
		if(flags != 0) {
			// Save current connection
			ClientConnection = connection;
		}
    }

    if(gattdb_adc == attribute) {
		if(flags != 0) {
			// Save current connection
			ClientConnection = connection;
			GpioAdcDac_start();
		} else {
			GpioAdcDac_stop();
		}
    }

}

/**
 * @brief      Function that is called when the characteristic status is changed.
 * @param[in]  uint16_t attribute - Attribute handle.
 * @param[in]  uint8array *writeValue - Pointer to attribute value.
 * @retval     None.
 */
void GpioAdcDac_attrWrite(uint16_t attribute, uint8array *writeValue)
{

    if(gattdb_set_config == attribute) {

        if(!writeValue || writeValue->len < 4) {
            return;
        }

        if(writeValue->data[0] == 0x20) {
            // Toggle Red LED
            RedState = !RedState;
            GpioAdcDac_setLedRed(RedState);

            Buzzer_on();
            // Turn off buzzer after some time
            gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(100), EVT_TIMER_GPIO_ADC_BUZZER_OFF, true);

            // Send notification
            GpioAdcDac_sendLedState();

            TRACE_INFO_WP("Led: %s\n", RedState ? "ON" : "OFF");
        }
        else if(writeValue->data[0] == 0x01) {

            uint16_t l_dacValue = (writeValue->data[3] << 8) | writeValue->data[2];
            // Set custom SPI mode
            GpioAdcDac_setSpiMode(&SpiModeAD5592R);
            AD5592R_dacWriteData(Driver, DAC_PIN, l_dacValue);
            // Restore default SPI mode
            GpioAdcDac_setSpiMode(&SpiModeDefault);

            TRACE_INFO_WP("Set DAC: 0x%04X\n", l_dacValue);
        }
    }
}

/**
 * @brief      Function that is called when the software timer is triggered.
 * @param[in]  uint16_t timId - Timer ID (@ref ApplicationTimer_t).
 * @retval     None.
 */
void GpioAdcDac_timHandler(uint16_t timId)
{
    if(EVT_TIMER_GPIO_ADC == timId) {
        GpioAdcDac_sendAdcData();
    }
    if(EVT_TIMER_GPIO_ADC_BUZZER_OFF == timId) {
    	Buzzer_off();
    }
}

/**
 * @brief      Set the chip outputs to the tristate.
 * @retval     None.
 */
void GpioAdcDac_triState(void)
{
    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    AD5592R_confPin(Driver, LED_RED_PIN, AD5592R_PIN_MODE_TRISTATE);
    AD5592R_confPin(Driver, LED_GREEN_PIN, AD5592R_PIN_MODE_TRISTATE);
    AD5592R_confPin(Driver, OUT0_PIN, AD5592R_PIN_MODE_TRISTATE);
    AD5592R_confPin(Driver, OUT1_PIN, AD5592R_PIN_MODE_TRISTATE);
    AD5592R_confPin(Driver, IN0_PIN, AD5592R_PIN_MODE_TRISTATE);
    AD5592R_confPin(Driver, IN1_PIN, AD5592R_PIN_MODE_TRISTATE);
    AD5592R_confPin(Driver, ADC_PIN, AD5592R_PIN_MODE_TRISTATE);
    AD5592R_confPin(Driver, DAC_PIN, AD5592R_PIN_MODE_TRISTATE);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);
}

/**
 * @brief      Put chip to power down mode, but does not set the chip outputs to the tristate. For example, when the close connection.
 * @retval     None.
 */
void GpioAdcDac_off(void)
{
    GpioAdcDac_stop();
}

/**
 * @brief      Set red led state.
 * @param[in]  bool state - New led state.
 * @retval     None.
 */
void GpioAdcDac_setLedRed(bool state)
{

    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    AD5592R_gpioWriteData(Driver, LED_RED_PIN, state);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);

}

/**
 * @brief      Set green led state.
 * @param[in]  bool state - New led state.
 * @retval     None.
 */
void GpioAdcDac_setLedGreen(bool state)
{
    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    AD5592R_gpioWriteData(Driver, LED_GREEN_PIN, state);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);
}


/* Private functions. */

/**
 * @brief      Function that is called when to send a data. Event from the soft timer.
 * @retval     None.
 */
static void GpioAdcDac_sendAdcData(void)
{
    uint8_t l_buff[4];
    uint16_t l_size = sizeof(l_buff);

    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    uint16_t l_adcValue = 0;
    AD5592R_adcReadData(Driver, ADC_PIN, &l_adcValue);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);


    uint16_t l_time = (uint16_t)TimeUtils_getMs();
    memcpy(&l_buff[0], &l_time, 2);
    memcpy(&l_buff[2], &l_adcValue, 2);

    struct gecko_msg_gatt_server_send_characteristic_notification_rsp_t* resp;
    resp = gecko_cmd_gatt_server_send_characteristic_notification(ClientConnection, gattdb_adc, l_size, l_buff);

    if(resp->result) {
        TRACE_INFO_WP("GpioAdcDac_sendAdcData: error 0x%04X \n", resp->result);
    }

}

/**
 * @brief      Function that is called when LED state changed.
 * @retval     None.
 */
static void GpioAdcDac_sendLedState(void)
{
    uint8_t l_buff[3];
    uint16_t l_size = sizeof(l_buff);

    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    bool l_ledRed = false;

    AD5592R_gpioReadData(Driver, LED_RED_PIN, &l_ledRed);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);

    uint16_t l_time = (uint16_t)TimeUtils_getMs();
    memcpy(&l_buff[0], &l_time, 2);
    l_buff[2] = l_ledRed ? 1 : 0;


    struct gecko_msg_gatt_server_send_characteristic_notification_rsp_t* resp;
    resp = gecko_cmd_gatt_server_send_characteristic_notification(ClientConnection, gattdb_led_state, l_size, l_buff);

    if(resp->result) {
        TRACE_INFO_WP("GpioAdcDac_sendLedState: error 0x%04X \n", resp->result);
    }

}

/**
 * @brief      Set SPI mode for AD5592R.
 * @param[in]  SeSpi_mode_t *mode - New spi mode.
 * @retval     None.
 */
static void GpioAdcDac_setSpiMode(SeSpi_mode_t *mode)
{
    SeSpi_setMode(hSpi, mode);
}

/**
 * @brief      Default pin configuration.
 * @retval     None.
 */
static void GpioAdcDac_pinsDefaultConfig(void)
{
    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    AD5592R_confPin(Driver, LED_RED_PIN, AD5592R_PIN_MODE_GPIO_IN_OUT);
    AD5592R_confPin(Driver, LED_GREEN_PIN, AD5592R_PIN_MODE_GPIO_IN_OUT);
    AD5592R_confPin(Driver, OUT0_PIN, AD5592R_PIN_MODE_GPIO_IN_OUT);
    AD5592R_confPin(Driver, OUT1_PIN, AD5592R_PIN_MODE_GPIO_IN_OUT);
    AD5592R_confPin(Driver, IN0_PIN, AD5592R_PIN_MODE_GPIO_IN);
    AD5592R_confPin(Driver, IN1_PIN, AD5592R_PIN_MODE_GPIO_IN);
    AD5592R_confPin(Driver, ADC_PIN, AD5592R_PIN_MODE_ADC);
    AD5592R_confPin(Driver, DAC_PIN, AD5592R_PIN_MODE_DAC);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);
}

/**
 * @brief      Start software timer to GPIO and ADC measurement and sending the data stream to the client.
 * @retval     None.
 */
static void GpioAdcDac_start(void)
{
    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    AD5592R_powerDown(Driver, false);
    AD5592R_setIntReference(Driver, true);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);

    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(kSendPeriodMs), EVT_TIMER_GPIO_ADC, false);

    TRACE_INFO_WP("GpioAdcDac ON.\n");
}

/**
 * @brief      Stop software timer.
 * @retval     None.
 */
static void GpioAdcDac_stop(void)
{
    gecko_cmd_hardware_set_soft_timer(TIMER_STOP, EVT_TIMER_GPIO_ADC, true);

    // Set custom SPI mode
    GpioAdcDac_setSpiMode(&SpiModeAD5592R);

    AD5592R_powerDown(Driver, true);

    // Restore default SPI mode
    GpioAdcDac_setSpiMode(&SpiModeDefault);

    TRACE_INFO_WP("GpioAdcDac OFF.\n");
}
