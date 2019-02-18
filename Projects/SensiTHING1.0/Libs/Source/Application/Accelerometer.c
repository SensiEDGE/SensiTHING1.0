/***************************************************************************//**
 * @file   Accelerometer.c
 * @brief  The file defines the logic of the accelerometer.
 *         It is possible to independently send the data stream to the client and
 *         send a notification of a change in state (activity or inactivity detection).
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
#include "ADXL362.h"
#include "Accelerometer.h"


/* Local objects */
static ADXL362_t Adxl362;
static ADXL362_t *Driver = &Adxl362;

/* Pins used by the chip. */
static SeGpio_pin_t ChipSelectPin;
static SeGpio_pin_t ExtIntPin;

/* Data sending period. */
static const uint32_t kSendPeriodMs = 20;

/* Activity and inactivity detection settings. See the datasheet for the correct values. */
static const uint16_t kActivityThreshold = 100;
static const uint16_t kActivityTime = 10;

static const uint16_t kInactivityThreshold = 100;
static const uint16_t kInactivityTime = 500;

/* Connection ID. */
static uint8_t ClientConnection = 0;


/* Accelerometer measurement mode. */
typedef enum {
    ACC_STREAM = (1 << 0),
    ACC_ALERT  = (1 << 1),
} Accelerometer_measMode_t;

/* Accelerometer measurement mode mask. */
static uint32_t MeasMode = 0;

/* The flag that displays activity or inactivity state. */
static bool IsActive = false;


/* Private function prototypes */
static void Accelerometer_sendData(void);
static void Accelerometer_sendAlert(void);
static void Accelerometer_start(uint32_t modeMask);
static void Accelerometer_stop(uint32_t modeMask);
static void Accelerometer_extIntCallback(uint8_t pin);


/* Public functions */

/**
 * @brief      Initialize accelerometer module.
 * @param[in]  SeSpi_t *hspi - Pointer to the SPI with which ADXL362 works.
 * @retval     None.
 */
void Accelerometer_init(SeSpi_t *hspi)
{

    // Configuring chip select pin.
    ChipSelectPin.m_pin = BSP_ADXL362_CS_PIN;
    ChipSelectPin.m_port = BSP_ADXL362_CS_PORT;
    SeGpio_configPin(&ChipSelectPin, SE_GPIO_MODE_PUSH_PULL, true);

    // Configuring external interrupt pin
    ExtIntPin.m_pin = BSP_ADXL362_INT1_PIN;
    ExtIntPin.m_port = BSP_ADXL362_INT1_PORT;
    SeGpio_configExtIntPin(&ExtIntPin, SE_GPIO_MODE_INPUT_PULL_FILTER, true, SE_GPIO_INT_MODE_RISING, Accelerometer_extIntCallback);
    SeGpio_enableExtInterrupt(&ExtIntPin, true);

    if(!ADXL362_init(Driver, &ChipSelectPin, hspi)) {
        TRACE_INFO_WP("ADXL362_init FAIL\n");
        return;
    }

    ADXL362_setupActivityDetection(Driver, true, kActivityThreshold, kActivityTime);
    ADXL362_setupInactivityDetection(Driver, true, kInactivityThreshold, kInactivityTime);

    ADXL362_setIntMap1(Driver, ADXL362_INTMAP_ACT | ADXL362_INTMAP_INACT);

    ADXL362_setOutputRate(Driver, ADXL362_ODR_100_HZ);

    ADXL362_setPowerMode(Driver, ADXL362_PWR_STANDBY);

    TRACE_INFO_WP("ADXL362_init OK\n");

}

/**
 * @brief      Function that is called when the characteristic status is changed.
 * @param[in]  uint8_t connection - Connection ID.
 * @param[in]  uint16_t attribute - Attribute value.
 * @param[in]  uint16_t flags - New value of Client Characteristic Config.
 * @retval     None.
 */
void Accelerometer_charStatusChange(uint8_t connection, uint16_t attribute, uint16_t flags)
{

    Accelerometer_measMode_t l_mode;

    if(gattdb_acc_gyro_mag == attribute) {
        l_mode = ACC_STREAM;
    } else if(gattdb_accelerometer_events == attribute) {
        l_mode = ACC_ALERT;
    } else {
        return;
    }

    if(flags != 0) {
        // Save current connection
        ClientConnection = connection;
        Accelerometer_start(l_mode);
    } else {
        Accelerometer_stop(l_mode);
    }
}

/**
 * @brief      Function that is called when the software timer is triggered.
 * @param[in]  uint16_t timId - Timer ID (@ref ApplicationTimer_t).
 * @retval     None.
 */
void Accelerometer_timHandler(uint16_t timId)
{
    if(EVT_TIMER_ACCELEROMETER == timId) {
        Accelerometer_sendData();
    }
}

/**
 * @brief      Function that is called when an external interrupt occurs.
 * @param[in]  uint16_t signals - External signals (@ref ApplicationExtInt_t).
 * @retval     None.
 */
void Accelerometer_extIntHandler(uint16_t signals)
{
   if(signals & (1 << BSP_ADXL362_INT1_PIN)) {
       Accelerometer_sendAlert();
   }
}

/**
 * @brief      Turns off the accelerometer measurement. For example, when the close connection.
 * @retval     None.
 */
void Accelerometer_off(void)
{
    Accelerometer_stop(ACC_STREAM | ACC_ALERT);
}


/* Private functions */

/**
 * @brief      The function that is called when to send a data. Event from the soft timer.
 * @retval     None.
 */
static void Accelerometer_sendData(void)
{
    float l_acc[3] = {0};
    uint8_t l_buff[20];
    uint16_t l_size = 0;

    ADXL362_getGxyz(Driver, &l_acc[0], &l_acc[1], &l_acc[2]);

    uint16_t l_time = (uint16_t)TimeUtils_getMs();
    int16_t l_acc16[3];


    for(int i = 0; i < 3; i++) {
    	l_acc16[i] =  (int16_t)(l_acc[i] * 1000.0);
    }

	memcpy(&l_buff[0], &l_time, 2);
	memcpy(&l_buff[2], l_acc16, 6);
	memset(&l_buff[8], 0, 6);	// Not used
	memset(&l_buff[14], 0, 6);	// Not used
    l_size = 20;

    struct gecko_msg_gatt_server_send_characteristic_notification_rsp_t* resp;
    resp = gecko_cmd_gatt_server_send_characteristic_notification(ClientConnection, gattdb_acc_gyro_mag, l_size, l_buff);

    if(resp->result) {
        TRACE_INFO_WP("Accelerometer_sendData: error 0x%04X \n", resp->result);
    }
}

/**
 * @brief      The function that is called when detected an inactivity or activity condition.
 * @retval     None.
 */
static void Accelerometer_sendAlert(void)
{
    uint8_t l_alert = 0;
    ADXL362_statusReg_t l_status;
    uint8_t l_buff[4];
    uint8_t l_size = 0;

    ADXL362_getStatus(Driver, &l_status);

    if(IsActive != l_status.bits.Active) {
        IsActive = l_status.bits.Active;
        l_alert = (IsActive != 0) ? 0x80 : 0x00 ;

        uint16_t l_time = (uint16_t)TimeUtils_getMs();
        memcpy(&l_buff[0], &l_time, 2);
        l_buff[2] = 1;
        l_buff[3] = l_alert;
        l_size = 4;
        gecko_cmd_gatt_server_send_characteristic_notification(ClientConnection, gattdb_accelerometer_events, l_size, l_buff);
    }
}

/**
 * @brief      Start measure the acceleration.
 * @param[in]  uint32_t modeMask - Bit mask that represents which operating mode should be enable.
 * @retval     None.
 */
static void Accelerometer_start(uint32_t modeMask)
{
    if(!MeasMode) {
        ADXL362_setPowerMode(Driver, ADXL362_PWR_MEASURE);
        TRACE_INFO_WP("Accelerometer ON.\n");
    }

    if(modeMask & ACC_STREAM) {
        gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(kSendPeriodMs), EVT_TIMER_ACCELEROMETER, false);
    }

    MeasMode |= modeMask;

    // Clear interrupt flags by reading the STATUS register
    ADXL362_statusReg_t l_status;
    ADXL362_getStatus(Driver, &l_status);
}

/**
 * @brief      Stop measure the acceleration.
 * @param[in]  uint32_t modeMask - Bit mask that represents which operating mode should be disabled.
 * @retval     None.
 */
static void Accelerometer_stop(uint32_t modeMask)
{
    MeasMode &= ~modeMask;

    if(modeMask & ACC_STREAM) {
        gecko_cmd_hardware_set_soft_timer(TIMER_STOP, EVT_TIMER_ACCELEROMETER, true);
    }

    if(!MeasMode) {
        ADXL362_setPowerMode(Driver, ADXL362_PWR_STANDBY);
        TRACE_INFO_WP("Accelerometer OFF.\n");
    }
}

/**
 * @brief      Handler for external interrupt.
 * @param[in]  uint8_t pin - The pin index the callback function is invoked for.
 * @retval     None.
 */
static void Accelerometer_extIntCallback(uint8_t pin)
{
    if(pin == BSP_ADXL362_INT1_PIN )  {
        gecko_external_signal(1 << BSP_ADXL362_INT1_PIN);
    }
}
