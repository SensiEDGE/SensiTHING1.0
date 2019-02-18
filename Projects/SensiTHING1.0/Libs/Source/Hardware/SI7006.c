/***************************************************************************//**
 * @file   SI7006.c
 * @brief  Driver for Analog Devices SI7006 -  Humidity Sensor.
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

#include <string.h>

#include "SI7006.h"


/* Private defines and types. */

/* Device ID. */
#define SI7006_DEVICE_ID             0x06

/* SI7006 commands. */
typedef enum {
    SI7006_CMD_READ_RH              = 0xE5,
    SI7006_CMD_READ_TEMP            = 0xE3,
    SI7006_CMD_READ_TEMP_AFTER_RH   = 0xE0,

    SI7006_CMD_READ_RH_NH           = 0xF5,
    SI7006_CMD_READ_TEMP_NH         = 0xF3,

    SI7006_CMD_WRITE_USER           = 0xE6,
    SI7006_CMD_READ_USER            = 0xE7,

    SI7006_CMD_WRITE_HEATER         = 0x51,
    SI7006_CMD_READ_HEATER          = 0x11,

    SI7006_CMD_SOFTWARE_RESET       = 0xFE,

    SI7006_CMD_READ_ID1_1           = 0xFA,
    SI7006_CMD_READ_ID1_2           = 0x0F,
    SI7006_CMD_READ_ID2_1           = 0xFC,
    SI7006_CMD_READ_ID2_2           = 0xC9,

} SI7006_commands_t;


/* Private function prototypes. */
static bool SI7006_measurement(SI7006_t *handle, uint8_t cmd, uint16_t *pData);
static bool SI7006_getRegister(SI7006_t *handle, uint8_t cmd, uint8_t *reg);
static bool SI7006_setRegister(SI7006_t *handle, uint8_t cmd, uint8_t reg);
static bool SI7006_i2cTransfer(SI7006_t *handle, const uint8_t *pDataTx, uint16_t sizeTx,
                                uint8_t *pDataRx, uint16_t sizeRx);
static uint16_t SI7006_swapUint16(uint16_t value);
static float SI7006_calcRh(uint16_t);
static float SI7006_calcTemp(uint16_t);


/* Public functions. */

/**
 * @brief      Initialize SI7006.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[in]  SeI2c_t *hi2c - Pointer to the I2C with which SI7006 works.
 * @param[in]  uint8_t address - Chip address.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SI7006_init(SI7006_t *handle, SeI2c_t *hi2c, uint8_t address)
{
    if (!handle || !hi2c) {
        return false;
    }

    uint8_t l_cmdWrite[2];
    uint8_t l_dataRead[8];

    handle->m_address = address;
    handle->m_hi2c = hi2c;

    // Reset chip
    l_cmdWrite[0] = SI7006_CMD_SOFTWARE_RESET;
    SI7006_i2cTransfer(handle, l_cmdWrite, 1, l_dataRead, 0);
    WAIT(1);

    // Detect chip
    l_cmdWrite[0] = SI7006_CMD_READ_ID2_1;
    l_cmdWrite[1] = SI7006_CMD_READ_ID2_2;

    SI7006_i2cTransfer(handle, l_cmdWrite, sizeof(l_cmdWrite), l_dataRead, sizeof(l_dataRead));

    return (l_dataRead[0] == SI7006_DEVICE_ID);
}

/**
 * @brief      Read humidity.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[out] float *rh - Pointer to save humidity.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SI7006_getRh(SI7006_t *handle, float *rh)
{
    if (!handle || !rh) {
        return false;
    }

    bool l_result = false;
    uint16_t l_dataRead = 0;

    l_result = SI7006_measurement(handle, SI7006_CMD_READ_RH, &l_dataRead);

    if(l_result) {
        *rh = SI7006_calcRh(l_dataRead);
    }

    return l_result;
}

/**
 * @brief      Read temperature.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[out] float *temp - Pointer to save temperature.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SI7006_getTemp(SI7006_t *handle, float *temp)
{
    if (!handle || !temp) {
        return false;
    }

    bool l_result = false;
    uint16_t l_dataRead = 0;

    l_result = SI7006_measurement(handle, SI7006_CMD_READ_TEMP, &l_dataRead);

    if(l_result) {
        *temp = SI7006_calcTemp(l_dataRead);
    }

    return l_result;
}

/**
 * @brief      Read temperature after reading humidity. This function does not perform a measurement but
 *             returns the temperature value measured during the relative humidity measurement.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[out] float *temp - Pointer to save temperature.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SI7006_getTempAfterRh(SI7006_t *handle, float *temp)
{
    if (!handle || !temp) {
        return false;
    }

    bool l_result = false;
    uint16_t l_temp = 0;

    l_result = SI7006_measurement(handle, SI7006_CMD_READ_TEMP_AFTER_RH, &l_temp);

    if(l_result) {
        *temp = SI7006_calcTemp(l_temp);
     }

    return l_result;
}

/**
 * @brief      Enable/disable heater.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[in]  bool state - New state. 'true' - heater is enable, 'false' - disable.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SI7006_heater(SI7006_t *handle, bool state)
{
    if (!handle) {
        return false;
    }

    uint8_t l_userReg = 0;

    if(!SI7006_getRegister(handle, SI7006_CMD_READ_USER, &l_userReg)) {
        return false;
    }

    if(state) {
        l_userReg |= (1 << 2);
    } else {
        l_userReg &= ~(1 << 2);
    }

    return SI7006_setRegister(handle, SI7006_CMD_WRITE_USER, l_userReg);
}

/**
 * @brief      Set heater power level.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[in]  uint8_t level - Heater level. Values can be from 0 to 15. At power supply VDD = 3.3 V
 *             0 - 3.09 mA, 15 - 94.20 mA
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SI7006_heaterLevel(SI7006_t *handle, uint8_t level)
{
    if (!handle) {
        return false;
    }

    return  SI7006_setRegister(handle, SI7006_CMD_WRITE_HEATER, (level & 0x0F));
}


/* Private functions. */

/**
 * @brief      Get measurement.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[in]  uint8_t cmd - Target command.
 * @param[out] uint16_t *pData - Pointer to save the read data.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool SI7006_measurement(SI7006_t *handle, uint8_t cmd, uint16_t *pData)
{
    if (!handle || !pData) {
        return false;
    }

    bool l_result = false;
    uint8_t l_cmdWrite = cmd;
    uint16_t l_dataRead = 0;

    l_result = SI7006_i2cTransfer(handle, &l_cmdWrite, 1, (uint8_t*)&l_dataRead, 2);

    if(l_result) {
        *pData = SI7006_swapUint16(l_dataRead);
    }

    return l_result;
}

/**
 * @brief      Read register.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[in]  uint8_t cmd - Target command.
 * @param[out] uint8_t *reg - Pointer to save register value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool SI7006_getRegister(SI7006_t *handle, uint8_t cmd, uint8_t *reg)
{
    return SI7006_i2cTransfer(handle, &cmd, 1, reg, 1);
}

/**
 * @brief      Write register.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[in]  uint8_t cmd - Target command.
 * @param[in]  uint8_t reg - Register value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool SI7006_setRegister(SI7006_t *handle, uint8_t cmd, uint8_t reg)
{
    uint8_t l_cmdWrite[2] = {cmd, reg};

    return SI7006_i2cTransfer(handle, l_cmdWrite, 2, 0, 0);
}

/**
 * @brief      Write and read data from I2C.
 * @param[in]  SI7006_t *handle - Pointer to instance of SI7006.
 * @param[in]  const uint8_t *pDataTx - Pointer to transmit data buffer.
 * @param[in]  uint16_t sizeTx - Amount of data to be sent (in bytes).
 * @param[out] uint8_t *pDataRx - Pointer to receive data buffer.
 * @param[in]  uint16_t sizeRx - amount of data to be received (in bytes).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool SI7006_i2cTransfer(SI7006_t *handle, const uint8_t *pDataTx, uint16_t sizeTx,
                                uint8_t *pDataRx, uint16_t sizeRx)
{
    return SeI2c_transfer(handle->m_hi2c, handle->m_address, pDataTx, sizeTx, pDataRx, sizeRx);
}

/**
 * @brief      Swap bytes in a 16-bit variable.
 * @param[in]  uint16_t value - 16-bit value in which you need to change bytes in places.
 * @retval     uint16_t - 16-bit value with changed bytes by places.
 */
static uint16_t SI7006_swapUint16(uint16_t value)
{
    return ((value >> 8) & 0x00FF) | ((value & 0x00FF) << 8);
}

/**
 * @brief      Convert the raw humidity in a hex into a float.
 * @param[in]  uint16_t value - Raw humidity in hex.
 * @retval     float - Humidity in percent.
 */
static float SI7006_calcRh(uint16_t value)
{
    return ((125.0 * value)/65536) - 6;
}

/**
 * @brief      Convert the raw temperature in a hex into a float.
 * @param[in]  uint16_t value - Raw temperature in hex.
 * @retval     float - Temperature in degrees Celsius.
 */
static float SI7006_calcTemp(uint16_t value)
{
    return ((175.72 * value)/65536) - 46.85;
}
