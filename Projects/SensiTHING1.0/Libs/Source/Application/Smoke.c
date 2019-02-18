/***************************************************************************//**
 * @file   Smoke.c
 * @brief  The file defines the logic of the smoke sensor.
 *         The sensor detects the presence of smoke using two measurement
 *         channels operating in two timeslots. The data is sent to the
 *         client on readiness in the FIFO. To indicate the availability
 *         of data, the GPIO0 chip output is used.
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
#include "ADPD188BI.h"
#include "Smoke.h"


/* Local objects */
static ADPD188BI_t Adpd188BI;
static ADPD188BI_t * Driver = &Adpd188BI;

/* Pins used by the chip. */
//static SeGpio_pin_t ChipSelectPin;
//static SeGpio_pin_t LevelShiftOePin;
static SeGpio_pin_t ExtIntPin;
static SeGpio_pin_t enable5vPin;

/* Connection ID. */
static uint8_t ClientConnection = 0;


/* Private function prototypes */
static void Smoke_sendData(void);
static void Smoke_start(void);
static void Smoke_stop(void);
static void Smoke_extIntCallback(uint8_t pin);


/* Public functions */

/**
 * @brief      Initialize smoke module.
 * @param[in]  SeI2c_t *hi2c - Pointer to the I2C with which ADPD188BI works.
 * @retval     None
 */
void Smoke_init(SeI2c_t *hi2c)
{

    // Configuring chip select pin.
//    ChipSelectPin.m_pin = BSP_ADPD188BI_CS_PIN;
//    ChipSelectPin.m_port = BSP_ADPD188BI_CS_PORT;
//    SeGpio_configPin(&ChipSelectPin, SE_GPIO_MODE_PUSH_PULL, true);

    // Configuring external interrupt pin
    ExtIntPin.m_pin = BSP_ADPD188BI_GPIO0_PIN;
    ExtIntPin.m_port = BSP_ADPD188BI_GPIO0_PORT;
    SeGpio_configExtIntPin(&ExtIntPin, SE_GPIO_MODE_INPUT_PULL_FILTER, true, SE_GPIO_INT_MODE_RISING, Smoke_extIntCallback);
    SeGpio_enableExtInterrupt(&ExtIntPin, true);

    // Configure 5v power pin
    enable5vPin.m_pin = BSP_ADPD188BI_5V_PIN;
    enable5vPin.m_port = BSP_ADPD188BI_5V_PORT;
    SeGpio_configPin(&enable5vPin, SE_GPIO_MODE_OD, false);


//        // Config level shift IC OE pin                         // TODO: Delete pin
//        LevelShiftOePin.m_pin = BSP_ADPD188BI_LCS_PIN;
//        LevelShiftOePin.m_port = BSP_ADPD188BI_LCS_PORT;
//        SeGpio_configPin(&LevelShiftOePin, SE_GPIO_MODE_PUSH_PULL, true);

    // Init driver
    if(!ADPD188BI_init(Driver, hi2c, BSP_ADPD188BI_I2C_ADR)) {
        TRACE_INFO_WP("ADPD188BI_init FAIL\n");
        return;
    }

    ADPD188BI_setOperationMode(Driver, ADPD188BI_PROGRAM);

    ADPD188BI_setPdConfig(Driver, ADPD188BI_SLOT_A, ADPD188BI_PD_E1CH1_E2CH2_P1CH3_P2CH4);
    ADPD188BI_setPdConfig(Driver, ADPD188BI_SLOT_B, ADPD188BI_PD_E1CH1_E2CH2_P1CH3_P2CH4);

    // IR led
    ADPD188BI_setLedConfig(Driver, ADPD188BI_SLOT_A, ADPD188BI_LED_LEDX3);
    // Blue led
    ADPD188BI_setLedConfig(Driver, ADPD188BI_SLOT_B, ADPD188BI_LED_LEDX1);

    // Set average factor
    ADPD188BI_setAdcAvgFactor(Driver, ADPD188BI_SLOT_A, ADPD188BI_NUM_AVG_8);
    ADPD188BI_setAdcAvgFactor(Driver, ADPD188BI_SLOT_B, ADPD188BI_NUM_AVG_8);


    // FIFO is configured to store samples of sum of the 4 channels in a 16-bit format.
    ADPD188BI_setSlotsFifoConfig(Driver, ADPD188BI_SLOT_A, ADPD188BI_SUM_4CH_16);
    ADPD188BI_setSlotsFifoConfig(Driver, ADPD188BI_SLOT_B, ADPD188BI_SUM_4CH_16);

    // Config gpio pin
    ADPD188BI_setGpioConfig(Driver, ADPD188BI_GPIO0, ADPD188BI_GPIO_INTERRUPT);

    // The interrupt will be generated when there are 2 data words in the FIFO. One for each slot.
    ADPD188BI_enableFiFoInterrupts(Driver, 2);



    ADPD188BI_enableSlots(Driver, ADPD188BI_SLOT_A, true);
    ADPD188BI_enableSlots(Driver, ADPD188BI_SLOT_B, true);

    ADPD188BI_setOperationMode(Driver, ADPD188BI_STANDBY);

    TRACE_INFO_WP("ADPD188BI_init OK\n");

}

/**
 * @brief      Function that is called when the characteristic status is changed.
 * @param[in]  uint8_t connection - Connection ID.
 * @param[in]  uint16_t attribute - Attribute value.
 * @param[in]  uint16_t flags - New value of Client Characteristic Config.
 * @retval     None.
 */
void Smoke_charStatusChange(uint8_t connection, uint16_t attribute, uint16_t flags)
{
    if(gattdb_smoke == attribute) {
        if(flags != 0) {
            // Save current connection
            ClientConnection = connection;
            Smoke_start();
        } else {
            Smoke_stop();
        }
    }
}

/**
 * @brief      Function that is called when an external interrupt occurs.
 * @param[in]  uint16_t signals - External signals (@ref ApplicationExtInt_t).
 * @retval     None.
 */
void Smoke_extIntHandler(uint16_t signals)
{
   if(signals & (1 << BSP_ADPD188BI_GPIO0_PIN)) {
       Smoke_sendData();
   }
}

/**
 * @brief      Turns off the smoke measurement. For example, when the close connection.
 * @retval     None.
 */
void Smoke_off(void)
{
    Smoke_stop();
}


/* Private functions */

/**
 * @brief      Function that is called when to send a data.
 * @retval     None.
 */
static void Smoke_sendData(void)
{
    uint8_t l_buff[7] = {0};
    uint16_t l_size = 0;

    // We read values as words
    uint16_t l_fifoBuff[ADPD188BI_FIFO_BUF_MAX_SIZE/2];

    // FIFO is configured to store one sample of sum of the 4 channels in a 16-bit format.
    // Thus, there will be 2 data words in the FIFO. One for each slot.
    uint16_t l_fifoSamples = 2;
    ADPD188BI_readData16FromFifo(Driver, l_fifoBuff, ADPD188BI_FIFO_BUF_MAX_SIZE, l_fifoSamples);

    uint16_t l_valueChA = l_fifoBuff[0];
    uint16_t l_valueChB = l_fifoBuff[1];


    float div = 0;
    if(l_valueChA != 0){
        div = (float)l_valueChB/l_valueChA;
    }


    uint16_t l_time = (uint16_t)TimeUtils_getMs();
    memcpy(&l_buff[0], &l_time, 2);
    memcpy(&l_buff[2], &l_valueChA, 2);
    memcpy(&l_buff[4], &l_valueChB, 2);
    l_buff[6] = (div > 1.4);
    l_size = 7;

    struct gecko_msg_gatt_server_send_characteristic_notification_rsp_t* resp;
    resp = gecko_cmd_gatt_server_send_characteristic_notification(ClientConnection, gattdb_smoke, l_size, l_buff);

    if(resp->result) {
        TRACE_INFO_WP("Smoke_sendData error 0x%04X \n",resp->result);
    }
}

/**
 * @brief      Start sensor working.
 * @retval     None.
 */
static void Smoke_start(void)
{
	SeGpio_setOutPin(&enable5vPin, true);
    ADPD188BI_setOperationMode(Driver, ADPD188BI_NORMAL);
    TRACE_INFO_WP("Smoke ON.\n");
}

/**
 * @brief      Stop sensor working.
 * @retval     None.
 */
static void Smoke_stop(void)
{
    ADPD188BI_setOperationMode(Driver, ADPD188BI_STANDBY);
    SeGpio_setOutPin(&enable5vPin, false);
    TRACE_INFO_WP("Smoke OFF.\n");
}

/**
 * @brief      Handler for external interrupt.
 * @param[in]  uint8_t pin - The pin index the callback function is invoked for.
 * @retval     None.
 */
static void Smoke_extIntCallback(uint8_t pin)
{
    if(pin == BSP_ADPD188BI_GPIO0_PIN )  {
        gecko_external_signal(1 << BSP_ADPD188BI_GPIO0_PIN);
    }
}
