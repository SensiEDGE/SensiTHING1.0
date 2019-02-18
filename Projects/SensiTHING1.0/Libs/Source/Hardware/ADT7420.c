/***************************************************************************//**
 * @file   ADT7420.c
 * @brief  Driver for Analog Devices ADT7420 -  Temperature Sensor.
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

#include "ADT7420.h"


/* Private defines. */

/* Device ID. */
#define ADT7420_DEVICE_ID               0xCB

/* Maximum number of bytes to send. Used for send buffer. */
#define ADT7420_TX_PACKET_MAX_SIZE      2



/* Private function prototypes. */
static bool ADT7420_i2cWrite(ADT7420_t *handle, uint8_t reg, const uint8_t *pData, uint16_t size);
static bool ADT7420_i2cRead(ADT7420_t *handle, uint8_t reg, uint8_t *pData, uint16_t size);
static uint16_t ADT7420_swapUint16(uint16_t value);


/* Public functions. */

/**
 * @brief      Initialize ADT7420.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  SeI2c_t *hi2c - Pointer to the I2C with which ADT7420 works.
 * @param[in]  uint8_t address - Chip address.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_init(ADT7420_t *handle, SeI2c_t *hi2c, uint8_t address)
{
    if (!handle || !hi2c) {
        return false;
    }

    handle->m_address = address;
    handle->m_hi2c = hi2c;

    // Reset chip
    ADT7420_setRegisterValue(handle, ADT7420_REG_SOFTWARE_RESET, 0);
    WAIT(1);

    // Read chip ID
    uint8_t l_chipId = 0;
    ADT7420_getRegisterValue(handle, ADT7420_REG_ID, &l_chipId);

    return (l_chipId == ADT7420_DEVICE_ID);

}

/**
 * @brief      Set register value.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  ADT7420_registers_t reg - Register address.
 * @param[in]  uint8_t value - New register value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_setRegisterValue(ADT7420_t *handle, ADT7420_registers_t reg, uint8_t value)
{
    if (!handle) {
        return false;
    }

    bool l_result = ADT7420_i2cWrite(handle, reg, &value, 1);

    // Save resolution for converting the raw temperature in a hex into a float
    if(reg == ADT7420_REG_CONFIG && l_result) {
        handle->m_resolution = ((ADT7420_configReg_t)value).bits.Resolution;
    }

    return l_result;
}

/**
 * @brief      Get register value.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  ADT7420_registers_t reg - Register address.
 * @param[out] uint8_t *value - Pointer to save the read value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_getRegisterValue(ADT7420_t *handle, ADT7420_registers_t reg, uint8_t *value)
{
    if (!handle || !value) {
        return false;
    }

    return ADT7420_i2cRead(handle, reg, value, 1);
}

/**
 * @brief      Convert the raw temperature in a hex into a float.
 * @param[in]  uint16_t rawHex - Raw temperature in hex.
 * @param[in]  ADT7420_resolution_t resolution - Raw value resolution.
 * @retval     float - Temperature in degrees Celsius.
 */
float ADT7420_convertHexToDegrees (uint16_t rawHex, ADT7420_resolution_t resolution)
{
    float l_temp = 0.0;

    // Check the status of the temperature sign bit (MSB)
    if(( rawHex & 0x8000 ) == 0x8000) {
        // If sign bit is 1 use the negative temperature equation
        if(resolution == ADT7420_RESOLUTION_16BIT) {
            // 16-bit temperature word data
            l_temp = (((float)rawHex - 65536)/128);
        } else {
            // 13-bit temperature word data
            rawHex = rawHex >> 3;
            l_temp = (((float)rawHex - 8192)/16);
        }
    } else {
        // If sign bit is 0, use the positive temperature equation
        if(resolution == ADT7420_RESOLUTION_16BIT) {
            // 16-bit temperature word data
            l_temp = (float)rawHex/128;
        } else {
            // 13-bit temperature word data
            rawHex = rawHex >> 3;
            l_temp = ((float)rawHex/16);
        }
    }

    return l_temp;
}

/**
 * @brief      Convert the temperature in a float into a hex.
 * @param[in]  float temperature - Temperature in degrees Celsius.
 * @param[in]  ADT7420_resolution_t resolution - Raw value resolution.
 * @retval     uint16_t - Raw temperature in hex.
 */
uint16_t ADT7420_convertDegreesToHex (float temperature, ADT7420_resolution_t resolution)
{
    uint16_t l_degreeHex = 0;

    // If sign bit is 1 use the negative temperature equation
    if(resolution == ADT7420_RESOLUTION_16BIT) {
        // 16-bit temperature word data
        if(temperature < 0) {
            l_degreeHex = (int16_t)((temperature * 128.0) + 65536);
        } else {
            l_degreeHex = (int16_t)(temperature * 128.0);
        }
    } else {
        // 13-bit temperature word data
        if(temperature < 0) {
            l_degreeHex = (int16_t)(temperature * 16.0) + 8192;
            l_degreeHex = l_degreeHex << 3;
        } else {
            l_degreeHex = (int16_t)(temperature * 16.0);
            l_degreeHex = l_degreeHex << 3;
        }
    }

    return l_degreeHex;
}

/**
 * @brief      Set configuration.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  ADT7420_configReg_t config - Configuration register structure.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_setConfig(ADT7420_t *handle, ADT7420_configReg_t config)
{
    if (!handle) {
        return false;
    }

    return ADT7420_setRegisterValue(handle, ADT7420_REG_CONFIG, config.reg);
}

/**
 * @brief      Get configuration.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[out] ADT7420_configReg_t *config - Pointer to configuration register structure.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_getConfig(ADT7420_t *handle, ADT7420_configReg_t *config)
{
    if (!handle || !config) {
        return false;
    }

    return ADT7420_getRegisterValue(handle, ADT7420_REG_CONFIG, &config->reg);
}

/**
 * @brief      Get status.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[out] ADT7420_statusReg_t *status - Pointer to status register structure.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_getStatus(ADT7420_t *handle, ADT7420_statusReg_t *status)
{
    if (!handle || !status) {
        return false;
    }

    return ADT7420_getRegisterValue(handle, ADT7420_REG_STATUS, &status->reg);
}

/**
 * @brief      Check if new data is ready.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @retval     bool - 'true' if new data is available, 'false' - otherwise.
 */
bool ADT7420_dataReady(ADT7420_t *handle)
{
    ADT7420_statusReg_t status;

    return (ADT7420_getStatus(handle, &status) && (status.bits.RDY == 0));
}

/**
 * @brief      Set operation mode.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  ADT7420_operationMode_t mode - Target operation mode.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_setOperationMode(ADT7420_t *handle, ADT7420_operationMode_t mode)
{
    if (!handle) {
        return false;
    }

    ADT7420_configReg_t l_config;

    if(!ADT7420_getConfig(handle, &l_config)) {
        return false;
    }

    l_config.bits.OpMode = mode;

    return ADT7420_setConfig(handle, l_config);

}

/**
 * @brief      Get temperature.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[out] float *temperature - Pointer to save temperature.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_getTemperature(ADT7420_t *handle, float *temperature)
{
    if (!handle || !temperature) {
        return false;
    }

    uint16_t l_rawTemp = 0;

    if(!ADT7420_i2cRead(handle, ADT7420_REG_TEMP_MSB, (uint8_t*)&l_rawTemp, 2)) {
        return false;
    }

    l_rawTemp = ADT7420_swapUint16(l_rawTemp);

    *temperature = ADT7420_convertHexToDegrees(l_rawTemp, handle->m_resolution);

    return true;
}

/**
 * @brief      Set point of temperature.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  ADT7420_setpoint_t setpoint - Target setpoint.
 * @param[in]  float temperature - Target temperature.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_setPoint(ADT7420_t *handle, ADT7420_setpoint_t setpoint, float temperature)
{
    if (!handle) {
        return false;
    }

    ADT7420_registers_t l_reg;
    uint16_t l_rawTemp = 0;

    l_rawTemp = ADT7420_convertDegreesToHex(temperature, handle->m_resolution);

    l_rawTemp = ADT7420_swapUint16(l_rawTemp);

    switch (setpoint) {
        case ADT7420_SETPOINT_T_HIGH:
            l_reg = ADT7420_REG_T_HIGH_SETPOINT_MSB;
            break;
        case ADT7420_SETPOINT_T_LOW:
            l_reg = ADT7420_REG_T_LOW_SETPOINT_MSB;
            break;
        case ADT7420_SETPOINT_T_CRIT:
            l_reg = ADT7420_REG_T_CRIT_SETPOINT_MSB;
            break;
        default:
            return false;
    }

    return ADT7420_i2cWrite(handle, l_reg, (uint8_t*)&l_rawTemp, 2);

}

/**
 * @brief      Set hysteresis.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  uint8_t hyst - Target hysteresis. Value are possible in steps of 1°C from 0°C to 15°C.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADT7420_setHysteresis(ADT7420_t *handle, uint8_t hyst)
{
    if (!handle || hyst > 15) {
        return false;
    }

    return ADT7420_setRegisterValue(handle, ADT7420_REG_T_HYST_SETPOINT, hyst);
}


/* Private functions. */

/**
 * @brief      Write data to I2C.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  uint8_t reg - Address of the register.
 * @param[in]  const uint8_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be write.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADT7420_i2cWrite(ADT7420_t *handle, uint8_t reg, const uint8_t *pData, uint16_t size)
{
    if(size > ADT7420_TX_PACKET_MAX_SIZE) {
        return false;
    }

    uint8_t l_transmit[ADT7420_TX_PACKET_MAX_SIZE + 1];

    l_transmit[0] = reg;

    memcpy(&l_transmit[1], pData, size);

    return SeI2c_transmit(handle->m_hi2c, handle->m_address, l_transmit, size + 1);
}

/**
 * @brief      Read data from I2C.
 * @param[in]  ADT7420_t *handle - Pointer to instance of ADT7420.
 * @param[in]  uint8_t reg - Address of the register.
 * @param[out] uint16_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be read.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADT7420_i2cRead(ADT7420_t *handle, uint8_t reg, uint8_t *pData, uint16_t size)
{
    return SeI2c_transfer(handle->m_hi2c, handle->m_address, &reg, 1, pData, size);
}

/**
 * @brief      Swap bytes in a 16-bit variable.
 * @param[in]  uint16_t value - 16-bit value in which you need to change bytes in places.
 * @retval     uint16_t - 16-bit value with changed bytes by places.
 */
static uint16_t ADT7420_swapUint16(uint16_t value)
{
    return ((value >> 8) & 0x00FF) | ((value & 0x00FF) << 8);
}
