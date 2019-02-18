/***************************************************************************//**
 * @file   Application.c
 * @brief  The file contains application initialization and basic event processing.
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

#include "hal-config.h"
#include "retargetserial.h"

// Peripheral drivers
#include "SeLetim.h"
#include "SeI2c.h"
#include "SeSpi.h"
#include "SeGpio.h"

// Modules
#include "Buzzer.h"
#include "Battery.h"
#include "Environmental.h"
#include "Smoke.h"
#include "Accelerometer.h"
#include "GpioAdcDac.h"
#include "UserInterface.h"
#include "ApplicationTimer.h"
#include "Application.h"


/* Driver objects. */
static SeLetim_t tim;
static SeI2c_t i2c0;
static SeI2c_t i2c1;
static SeSpi_t spi1;

/* Flag for indicating DFU Reset must be performed. */
static uint8_t boot_to_dfu = 0;


/* Private function prototypes. */
static void Application_initBsp(void);
static void Application_initTim(void);
static void Application_initI2c0(void);
static void Application_initI2c1(void);
static void Application_initSpi1(void);


/* Public functions. */

/**
 * @brief      Initialize application.
 * @retval     None.
 */
void Application_init(void)
{

#if (TRACE_LEVEL != TRACE_LEVEL_NO_TRACE)
    /* Initialization debug UART. */
    RETARGET_SerialInit();
#endif

    TRACE_INFO_WP("\n%s\n",APP_DEVNAME_DEFAULT);

    TimeUtils_init();

    UserInterface_verifyStart();

    /* Initialization drivers */
    Application_initTim();
    Application_initI2c0();
    Application_initI2c1();
    Application_initSpi1();
}

/**
 * @brief      The event handler.
 * @param[in]  struct gecko_cmd_packet *evt - Event.
 * @retval     None.
 */
void Application_handleEvents(struct gecko_cmd_packet *evt)
{

    uint8_t l_connection = 0;
    uint16_t l_flags = 0;
    uint16_t l_attribute = 0;
    uint8array *l_writeValue = 0;
    uint16_t l_softTimId = 0;
    uint16_t l_extSignal = 0;


    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {


        /* This boot event is generated when the system boots up after reset.
         * Do not call any stack commands before receiving the boot event.
         * Here the system is set to start advertising immediately after boot procedure. */
        case gecko_evt_system_boot_id:
            /* Hardware initialization */
            Application_initBsp();

            /* Set advertising parameters. 100ms advertisement interval.
             * The first parameter is advertising set handle
             * The next two parameters are minimum and maximum advertising interval, both in
             * units of (milliseconds * 1.6).
             * The last two parameters are duration and maxevents left as default. */
            gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

            /* Start general advertising and enable connections */
            gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);

            break;

        /* Connection opened event */
        case gecko_evt_le_connection_opened_id:
            UserInterface_updConnectionState(true);
            break;

        case gecko_evt_le_connection_closed_id:

            UserInterface_updConnectionState(false);
            /* Stop all modules */
            Battery_off();
            Environmental_off();
            Smoke_off();
            Accelerometer_off();
            GpioAdcDac_off();

            /* Check if need to boot to dfu mode */
            if (boot_to_dfu) {
                /* Enter to DFU OTA mode */
                gecko_cmd_system_reset(2);
            } else {
                /* Restart advertising after client has disconnected */
                gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
            }

            break;

        /* Indicates the changed value of CCC or received characteristic confirmation */
        case gecko_evt_gatt_server_characteristic_status_id:

            l_connection = evt->data.evt_gatt_server_characteristic_status.connection;
            l_flags      = evt->data.evt_gatt_server_characteristic_status.client_config_flags;

            /* Check if changed client characteristic config */
            if(evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01) {
                /* Check attribute */
                l_attribute = evt->data.evt_gatt_server_attribute_value.attribute;

                Battery_charStatusChange(l_connection, l_attribute, l_flags);
                Environmental_charStatusChange(l_connection, l_attribute, l_flags);
                GpioAdcDac_charStatusChange(l_connection, l_attribute, l_flags);
                Smoke_charStatusChange(l_connection, l_attribute, l_flags);
                Accelerometer_charStatusChange(l_connection, l_attribute, l_flags);
            }

            break;

        /* Value of attribute changed from the local database by remote GATT client */
        case gecko_evt_gatt_server_attribute_value_id:
            /* Check attribute */
            l_attribute = evt->data.evt_gatt_server_attribute_value.attribute;
            l_writeValue = &evt->data.evt_gatt_server_attribute_value.value;

            Environmental_attrWrite(l_attribute, l_writeValue);
            GpioAdcDac_attrWrite(l_attribute, l_writeValue);

            /* Send response to Write Request */
            gecko_cmd_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection, gattdb_ota_control, bg_err_success);

            break;

        /* Software Timer event */
        case gecko_evt_hardware_soft_timer_id:

            /* Check which software timer handle is in question */
            l_softTimId = evt->data.evt_hardware_soft_timer.handle;

            Battery_timHandler(l_softTimId);
            Environmental_timHandler(l_softTimId);
            GpioAdcDac_timHandler(l_softTimId);
            Accelerometer_timHandler(l_softTimId);
            UserInterface_timHandler(l_softTimId);

            break;

        /* External interrupts event */
        case gecko_evt_system_external_signal_id:
            /* Check which external interrupt handle is in question */

            l_extSignal = evt->data.evt_system_external_signal.extsignals;

            Environmental_extIntHandler(l_extSignal);
            Smoke_extIntHandler(l_extSignal);
            Accelerometer_extIntHandler(l_extSignal);

            break;


        /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

        /* Check if the user-type OTA Control Characteristic was written.
         * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
        case gecko_evt_gatt_server_user_write_request_id:

            TRACE_INFO_WP("Write %d, %d\n", evt->data.evt_gatt_server_user_write_request.characteristic, evt->data.evt_gatt_server_user_write_request.value.data[0]);

            if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
                /* Set flag to enter to OTA mode */
                boot_to_dfu = 1;
                /* Send response to Write Request */
                gecko_cmd_gatt_server_send_user_write_response(
                        evt->data.evt_gatt_server_user_write_request.connection, gattdb_ota_control, bg_err_success);

                /* Close connection to enter to DFU OTA mode */
                gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
            }

            break;

        default:
            break;
    }

}


/* Private functions */

/**
 * @brief      Initialization of the board support periphery.
 * @retval     None.
 */
static void Application_initBsp(void)
{
    /* Unique device ID */
    uint16_t devId;
    struct gecko_msg_system_get_bt_address_rsp_t* btAddr;
    char devName[APP_DEVNAME_LEN + 1];

    /* Init device name */
    /* Get the unique device ID */

    /* Create the device name based on the 16-bit device ID */
    btAddr = gecko_cmd_system_get_bt_address();
    devId = *((uint16*)(btAddr->address.addr));
    snprintf(devName, APP_DEVNAME_LEN + 1, APP_DEVNAME, devId);
    gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(devName), (uint8_t *)devName);


    /* Initialization modules */

    // Config CS for W25Q80 so as not to interfere with other chips.
    SeGpio_pin_t ChipSelectPinW25Q80 = {
            .m_pin = BSP_W25Q80_CS_PIN,
            .m_port = BSP_W25Q80_CS_PORT,
    };
    SeGpio_configPin(&ChipSelectPinW25Q80, SE_GPIO_MODE_PUSH_PULL, true);

    Buzzer_init(&tim);
    Battery_init(&i2c0);
    Environmental_init(&i2c1);
    GpioAdcDac_init(&spi1);
    Smoke_init(&i2c1);
    Accelerometer_init(&spi1);

    /* Initialization  user interface */
    UserInterface_init();

    TRACE_INFO_WP("\n%s started\n",devName);

}

/**
 * @brief      Initialization timer.
 * @retval     None.
 */
static void Application_initTim(void)
{
    SeLetim_init_t l_init;

    l_init.m_port = BSP_LETIM_PORT;
    l_init.m_ch0Loc = BSP_LETIM_CH0_LOC;
    l_init.m_polarity = true;

    SeLetim_init(&tim, &l_init);
}

/**
 * @brief      Initialization I2C.
 * @retval     None.
 */
static void Application_initI2c0(void)
{
    SeI2c_init_t l_init;

    l_init.m_port = BSP_I2C0_PORT;
    l_init.m_sclLoc = BSP_I2C0_SCL_LOC;
    l_init.m_sdaLoc = BSP_I2C0_SDA_LOC;

    l_init.m_i2cRefFreq = 0;
    l_init.m_i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
    l_init.m_i2cClhr = _I2C_CTRL_CLHR_STANDARD;

    SeI2c_init(&i2c0, &l_init);
}

/**
 * @brief      Initialization I2C.
 * @retval     None.
 */
static void Application_initI2c1(void)
{
    SeI2c_init_t l_init;

    l_init.m_port = BSP_I2C1_PORT;
    l_init.m_sclLoc = BSP_I2C1_SCL_LOC;
    l_init.m_sdaLoc = BSP_I2C1_SDA_LOC;

    l_init.m_i2cRefFreq = 0;
    l_init.m_i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
    l_init.m_i2cClhr = _I2C_CTRL_CLHR_STANDARD;

    SeI2c_init(&i2c1, &l_init);
}

/**
 * @brief      Initialization SPI.
 * @retval     None.
 */
static void Application_initSpi1(void)
{
    SeSpi_init_t l_init;

    l_init.m_port = BSP_SPI1_PORT;
    l_init.m_clkLoc = BSP_SPI1_SCK_LOC;
    l_init.m_misoLoc = BSP_SPI1_MIS0_LOC;
    l_init.m_mosiLoc = BSP_SPI1_MOSI_LOC;

    // Set default mode
    l_init.m_mode.m_baudrate = 1000000;
    l_init.m_mode.m_clockMode = spidrvClockMode0;
    l_init.m_mode.m_frameLength = 8;
    l_init.m_mode.m_bitOrder = spidrvBitOrderMsbFirst;

    SeSpi_init(&spi1, &l_init);
}

