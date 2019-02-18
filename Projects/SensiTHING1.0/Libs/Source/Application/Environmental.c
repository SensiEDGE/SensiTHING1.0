/***************************************************************************//**
 * @file   Environmental.c
 * @brief  The module allows you to measure the ambient temperature and humidity
 *         and send notifications if the temperature is outside the specified
 *         limits. The user can set these limits from the phone.
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
#include "SeGpio.h"
#include "ADT7420.h"
#include "SI7006.h"
#include "Environmental.h"


/* Default limits settings. */
static const int8_t kTempAlertMin = 20;
static const int8_t kTempAlertMax = 40;
static const uint8_t kTempHysteresis = 1;

/* Data sending period. */
static const uint32_t kSendPeriodMs = 1000;

/* Local objects */
static ADT7420_t Adt7420;
static ADT7420_t * TempDriver = &Adt7420;

static SI7006_t Si7006;
static SI7006_t * HumDriver = &Si7006;

/* Pins used by the chip. */
static SeGpio_pin_t ExtIntPin;

/* Connection ID. */
static uint8_t ClientConnection = 0;

/* Stored temerature value. */
static float TemperatureValue = 0;

/* Thermometer measurement mode. */
typedef enum {
    TEMP_STREAM = (1 << 0),
    TEMP_ALERT  = (1 << 1),
} Thermometer_measMode_t;

/* Thermometer measurement mode mask. */
static uint32_t MeasMode = 0;


/* Private function prototypes */
static void Environmental_start(uint32_t modeMask);
static void Environmental_stop(uint32_t modeMask);
static void Environmental_sendData(void);
static void Environmental_sendTemperatureAlert(void);
static void Environmental_extIntCallback(uint8_t pin);


/* Public functions. */

/**
 * @brief      Initialize environmental module.
 * @param[in]  SeI2c_t *hi2c - Pointer to the I2C with which ADT7420 works.
 * @retval     None
 */
void Environmental_init(SeI2c_t *hi2c)
{

	// Init humidity sensor
    if(!SI7006_init(HumDriver, hi2c, BSP_SI7006_I2C_ADR)) {
        TRACE_INFO_WP("SI7006_init FAIL\n");
        return;
    }
    TRACE_INFO_WP("SI7006_init OK\n");


    // Init temperature sensor
    // Configuring external interrupt pin
    ExtIntPin.m_pin = BSP_ADT7420_INT_PIN;
    ExtIntPin.m_port = BSP_ADT7420_INT_PORT;
    SeGpio_configExtIntPin(&ExtIntPin, SE_GPIO_MODE_INPUT_PULL_FILTER, true, SE_GPIO_INT_MODE_RISING_FALLING, Environmental_extIntCallback);
    SeGpio_enableExtInterrupt(&ExtIntPin, true);

    if(!ADT7420_init(TempDriver, hi2c, BSP_ADT7420_I2C_ADR)) {
        TRACE_INFO_WP("ADT7420_init FAIL\n");
        return;
    }

    // Configuring chip
    ADT7420_setPoint(TempDriver, ADT7420_SETPOINT_T_LOW,  (float)(kTempAlertMin));
    ADT7420_setPoint(TempDriver, ADT7420_SETPOINT_T_HIGH, (float)(kTempAlertMax));
    ADT7420_setHysteresis(TempDriver, kTempHysteresis);

    ADT7420_configReg_t l_config;
    l_config.reg = 0;
    l_config.bits.Resolution = ADT7420_RESOLUTION_16BIT;
    l_config.bits.IntCtMode = ADT7420_INT_COMPARATOR;

    ADT7420_setConfig(TempDriver, l_config);

    // Shutdown chip at startup
    ADT7420_setOperationMode(TempDriver, ADT7420_OP_SHUTDOWN);

    TRACE_INFO_WP("ADT7420_init OK\n");

}

/**
 * @brief      Get temperature in degrees Celsius.
 * @retval     float - Measurement temperature.
 */
float Environmental_getTemperature(void)
{
    if (ADT7420_dataReady(TempDriver)) {
        ADT7420_getTemperature(TempDriver, &TemperatureValue);
    }

    return TemperatureValue;
}

/**
 * @brief      Get humidity in percent.
 * @retval     float - Measurement humidity.
 */
float Environmental_getHumidity(void)
{
    float l_hum;

    SI7006_getRh(HumDriver, &l_hum);
    return l_hum;
}

/**
 * @brief      Function that is called when the characteristic status is changed.
 * @param[in]  uint8_t connection - Connection ID.
 * @param[in]  uint16_t attribute - Attribute value.
 * @param[in]  uint16_t flags - New value of Client Characteristic Config.
 * @retval     None.
 */
void Environmental_charStatusChange(uint8_t connection, uint16_t attribute, uint16_t flags)
{

    Thermometer_measMode_t l_mode;

    if(gattdb_environmental == attribute) {
        l_mode = TEMP_STREAM;
    } else if(gattdb_temperature_alert == attribute) {
        l_mode = TEMP_ALERT;
    } else {
        return;
    }

    if(flags != 0) {
        // Save current connection
        ClientConnection = connection;
        Environmental_start(l_mode);
    } else {
    	Environmental_stop(l_mode);
    }
}

/**
 * @brief      Function that is called when the attribute value is changed.
 * @param[in]  uint16_t attribute - Attribute handle.
 * @param[in]  uint8array *writeValue - Pointer to attribute value.
 * @retval     None.
 */
void Environmental_attrWrite(uint16_t attribute, uint8array *writeValue)
{
    if(gattdb_set_config == attribute) {

        if(!writeValue || writeValue->len < 4) {
            return;
        }

        if(writeValue->data[0] == 0x01) {
            ADT7420_setPoint(TempDriver, ADT7420_SETPOINT_T_LOW,  (float)(writeValue->data[2]));
            ADT7420_setPoint(TempDriver, ADT7420_SETPOINT_T_HIGH, (float)(writeValue->data[3]));

            TRACE_INFO_WP("Alert limits: (%d - %d)\n", writeValue->data[2], writeValue->data[3]);
        }
    }

}

/**
 * @brief      Function that is called when the software timer is triggered.
 * @param[in]  uint16_t timId - Timer ID (@ref ApplicationTimer_t).
 * @retval     None.
 */
void Environmental_timHandler(uint16_t timId)
{
    if(EVT_TIMER_ENVIRONMENTAL == timId) {
    	Environmental_sendData();
    }
}

/**
 * @brief      Function that is called when an external interrupt occurs.
 * @param[in]  uint16_t signals - External signals (@ref ApplicationExtInt_t).
 * @retval     None.
 */
void Environmental_extIntHandler(uint16_t signals)
{
   if(signals & (1 << BSP_ADT7420_INT_PIN)) {
	   Environmental_sendTemperatureAlert();
   }
}

/**
 * @brief      Turns off the temperature and humidity measurement. For example, when the close connection.
 * @retval     None.
 */
void Environmental_off(void)
{
	Environmental_stop(TEMP_STREAM | TEMP_ALERT);
}


/* Private functions. */

/**
 * @brief      Awakes the chip and start software timer to measurement temperature and humidity.
 * @retval     None.
 */
static void Environmental_start(uint32_t modeMask)
{
    if(!MeasMode) {
        ADT7420_setOperationMode(TempDriver, ADT7420_OP_SPS);
        TRACE_INFO_WP("Environmental ON.\n");
    }

    if(modeMask & TEMP_STREAM) {
        gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(kSendPeriodMs), EVT_TIMER_ENVIRONMENTAL, false);
    }

    MeasMode |= modeMask;
}

/**
 * @brief      Stop software timer and shutdown chip.
 * @retval     None.
 */
static void Environmental_stop(uint32_t modeMask)
{
    MeasMode &= ~modeMask;

    if(modeMask & TEMP_STREAM) {
        gecko_cmd_hardware_set_soft_timer(TIMER_STOP, EVT_TIMER_ENVIRONMENTAL, true);
    }

    if(!MeasMode) {
        ADT7420_setOperationMode(TempDriver, ADT7420_OP_SHUTDOWN);
        TRACE_INFO_WP("Environmental OFF.\n");
    }
}

/**
 * @brief      Function that is called when to send a data. Send temperature to client.
 * @retval     None.
 */
static void Environmental_sendData(void)
{
    uint8_t l_buff[10];
    uint16_t l_size = 0;

    uint16_t l_time = (uint16_t)TimeUtils_getMs();
    uint16_t l_temp = (uint16_t)(Environmental_getTemperature() * 10);

    float l_rh = Environmental_getHumidity();
    uint16_t l_hum = (uint16_t)(10*l_rh);

    memcpy(&l_buff[0], &l_time, 2);
	memset(&l_buff[2], 0, 4);
	memcpy(&l_buff[6], &l_hum, 2);
	memcpy(&l_buff[8], &l_temp, 2);
	l_size = 10;


    gecko_cmd_gatt_server_send_characteristic_notification(ClientConnection, gattdb_environmental, l_size, l_buff);
}

/**
 * @brief      The function that is called when a temperature is outside the specified limits.
 * @retval     None.
 */
static void Environmental_sendTemperatureAlert(void)
{
    uint8_t l_alert = !SeGpio_getInPin(&ExtIntPin);
    gecko_cmd_gatt_server_send_characteristic_notification(ClientConnection, gattdb_temperature_alert, 1, &l_alert);
}

/**
 * @brief      Handler for external interrupt.
 * @param[in]  uint8_t pin - The pin index the callback function is invoked for.
 * @retval     None.
 */
static void Environmental_extIntCallback(uint8_t pin)
{
    if(pin == BSP_ADT7420_INT_PIN )  {
        gecko_external_signal(1 << BSP_ADT7420_INT_PIN);
    }
}
