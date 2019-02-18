/***************************************************************************//**
 * @file   Battery.c
 * @brief  The module sends the data stream to the client.
 *         The data contains battery voltage, capacity, and charging status.
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
#include "ExpSmoothFilter.h"
#include "Battery.h"
#include "LTC2942.h"


/* Local objects */
static LTC2942_t LTC2942;
static LTC2942_t * Driver = &LTC2942;


/* Data sending period. */
static const uint32_t kSendPeriodMs = 1000;


/* Connection ID. */
static uint8_t ClientConnection = 0;

static float AccChargeValue = 0;
static float PreviousAccChargeValue = 0;
static bool IsCharging = false;
static float CurrentMilliAmpers = 0;

/* Private function prototypes */
static void Battery_sendData(void);
static float Battery_getChargeLevel(void);
static float Battery_read(void);
static void Battery_start(void);
static void Battery_stop(void);


/* Public functions. */

/**
 * @brief      Initialize battery module.
 * @param[in]  SeI2c_t *hi2c - Pointer to a I2C driver.
 * @retval     None.
 */
void Battery_init(SeI2c_t *hi2c)
{
    if(!LTC2942_init(Driver, hi2c, BSP_LTC2942_I2C_ADR)) {
		TRACE_INFO_WP("LTC2942_init FAIL\n");
		return;
    }

	// Enable auto measurement of battery voltage and temperature
	LTC2942_setADCMode(Driver, LTC2942_ADC_AUTO);


	// Enable analog section of the chip (in case if it disabled)
	LTC2942_setAnalog(Driver, LTC2942_AN_ENABLED);

	// Set prescaler M value
	LTC2942_setPrescaler(Driver, LTC2942_PSCM_4);

	// Disable AL/CC pin
	LTC2942_setALCCMode(Driver, LTC2942_ALCC_DISABLED);

	LTC2942_setADCMode(Driver, LTC2942_ADC_SLEEP);
	LTC2942_setAnalog(Driver, LTC2942_AN_DISABLED);

    TRACE_INFO_WP("LTC2942_init OK\n");


}

/**
 * @brief      Get battery voltage.
 * @retval     float - Measurement voltage.
 */
float Battery_getVoltage(void)
{
    return Battery_read();
}

bool Battery_isCharging(void)
{
    return IsCharging;
}

/**
 * @brief      Get battery capacity.
 * @retval     float - Measurement battery capacity.
 */
float Battery_getCapacity(void)
{
    return AccChargeValue / 655.35;
}

/**
 * @brief      Function that is called when the characteristic status is changed.
 * @param[in]  uint8_t connection - Connection ID.
 * @param[in]  uint16_t attribute - Attribute value.
 * @param[in]  uint16_t flags - New value of Client Characteristic Config.
 * @retval     None.
 */
void Battery_charStatusChange(uint8_t connection, uint16_t attribute, uint16_t flags)
{
    if(gattdb_battery_status == attribute) {
        if(flags != 0) {
            // Save current connection
            ClientConnection = connection;

            Battery_start();

            // Init charge value
            uint16 l_ac = 0;
            LTC2942_getAC(Driver, &l_ac);
            PreviousAccChargeValue = AccChargeValue = (float)l_ac;
        } else {
            Battery_stop();
        }
    }
}

/**
 * @brief      Function that is called when the software timer is triggered.
 * @param[in]  uint16_t timId - Timer ID (@ref ApplicationTimer_t).
 * @retval     None.
 */
void Battery_timHandler(uint16_t timId)
{
    if(EVT_TIMER_BATTERY == timId) {
        // update filter value
        AccChargeValue = Battery_getChargeLevel();

        IsCharging = AccChargeValue - PreviousAccChargeValue > 0.1;

        // 1 Lsb = 1.328 uAH
        CurrentMilliAmpers = (PreviousAccChargeValue - AccChargeValue) * 1.328 * (3.6 * kSendPeriodMs)/1000;

        // save previous value
        PreviousAccChargeValue = AccChargeValue;

        Battery_sendData();
    }
}

/**
 * @brief      Turns off the battery measurement. For example, when the close connection.
 * @retval     None.
 */
void Battery_off(void)
{
    Battery_stop();
}


/* Private functions. */

/**
 * @brief      The function that is called when to send a data. Event from the soft timer.
 * @retval     None.
 */
static void Battery_sendData(void)
{
    uint8_t l_buff[9];
    uint16_t l_size = 0;

    float l_realVoltage = Battery_getVoltage();
    uint16_t l_time = (uint16_t)TimeUtils_getMs();
    uint16_t l_voltage = (uint16_t)(l_realVoltage * 1000);
    uint16_t l_capacity = (uint16_t)(10* Battery_getCapacity());
    uint16_t l_carrent = CurrentMilliAmpers;
    uint8_t  l_charging = IsCharging ? 2 : 1;


    memcpy(&l_buff[0], &l_time, 2);
    memcpy(&l_buff[2], &l_capacity, 2);
    memcpy(&l_buff[4], &l_voltage, 2);
    memcpy(&l_buff[6], &l_carrent, 2);
    memcpy(&l_buff[8], &l_charging, 1);
    l_size = 9;

    gecko_cmd_gatt_server_send_characteristic_notification(ClientConnection, gattdb_battery_status, l_size, l_buff);

}

static float Battery_getChargeLevel(void)
{
    uint16 l_ac = 0;
    LTC2942_getAC(Driver, &l_ac);
    return AccChargeValue + 0.3 * ((float)l_ac - AccChargeValue);
}

/**
 * @brief      Read battery voltage from ADC.
 * @retval     float - Battery voltage.
 */
static float Battery_read(void)
{
	float l_voltage = 0;

	LTC2942_getVoltage(Driver, &l_voltage);
	//TRACE_INFO_WP("V %f\n",l_voltage);
    return l_voltage;
}

/**
 * @brief      Start software timer to measurement battery voltage.
 * @retval     None.
 */
static void Battery_start(void)
{
	LTC2942_setAnalog(Driver, LTC2942_AN_ENABLED);
	LTC2942_setADCMode(Driver, LTC2942_ADC_AUTO);
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(kSendPeriodMs), EVT_TIMER_BATTERY, false);
    TRACE_INFO_WP("Battery ON.\n");
}

/**
 * @brief      Stop software timer.
 * @retval     None.
 */
static void Battery_stop(void)
{
    gecko_cmd_hardware_set_soft_timer(TIMER_STOP, EVT_TIMER_BATTERY, true);
    LTC2942_setADCMode(Driver, LTC2942_ADC_SLEEP);
    LTC2942_setAnalog(Driver, LTC2942_AN_DISABLED);
    TRACE_INFO_WP("Battery OFF.\n");
}
