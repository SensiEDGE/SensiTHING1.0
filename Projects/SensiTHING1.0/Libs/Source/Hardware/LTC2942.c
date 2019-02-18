/***************************************************************************//**
 * @file   LTC2942.c
 * @brief  Battery Driver.
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

#include "LTC2942.h"


/* Device ID. */
#define LTC2942_DEVICE_ID             0x00


/* Private function prototypes. */
static bool LTC2942_i2cTransfer(LTC2942_t *handle, const uint8_t *pDataTx, uint16_t sizeTx, uint8_t *pDataRx, uint16_t sizeRx);
static bool LTC2942_getRegister(LTC2942_t *handle, uint8_t cmd, uint8_t *reg);
static bool LTC2942_setRegister(LTC2942_t *handle, uint8_t cmd, uint8_t reg);
static uint16_t LTC2942_swapUint16(uint16_t value);



/* Public functions. */

/**
 * @brief      Initialize LTC2942.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[in]  SeI2c_t *hi2c - Pointer to the I2C with which LTC2942 works.
 * @param[in]  uint8_t address - Chip address.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool LTC2942_init(LTC2942_t *handle, SeI2c_t *hi2c, uint8_t address)
{
    if (!handle || !hi2c) {
        return false;
    }

    handle->m_address = address;
    handle->m_hi2c = hi2c;

    // Detect chip
    uint8_t l_chipId = 0;

    if(!LTC2942_getRegister(handle, LTC2942_REG_STATUS, &l_chipId)) {
    	return false;
    }

    return ((l_chipId & LTC2942_STATUS_CHIPID) == LTC2942_DEVICE_ID);
    return true;
}

/**
 * @brief      Get voltage.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[out] float *voltage - Pointer to save voltage.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool LTC2942_getVoltage(LTC2942_t *handle, float *voltage)
{
    if (!handle || !voltage) {
        return false;
    }

    bool l_result = false;
    uint16_t l_dataRead = 0;

	uint32_t l_value = 0;

	// Send voltage MSB register address
	uint8_t reg = LTC2942_REG_VOL_H;


	l_result = LTC2942_i2cTransfer(handle, &reg, 1, (uint8_t*)&l_dataRead, sizeof(l_dataRead));

	if(l_result) {
		// Calculate voltage
		l_value = LTC2942_swapUint16(l_dataRead) * 6000;
		l_value /= 65535;
		*voltage = l_value/1000.0;
	}

	return l_result;
}

/**
 * @brief      Configure ADC mode.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[in]  uint8_t mode - new ADC mode (one of LTC2942_ADC_XXX values).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool LTC2942_setADCMode(LTC2942_t *handle, uint8_t mode)
{
	// Read CONTROL register, clear ADC mode bits and configure new value
	uint8_t l_readReg = 0;

	if(!LTC2942_getRegister(handle, LTC2942_REG_CONTROL, &l_readReg)) {
		return false;
	}

	if(mode == LTC2942_ADC_SLEEP) {
		mode  = 1;
	}
	//l_readReg &= LTC2942_CTL_ADC_MSK;
	l_readReg &= LTC2942_CTL_SHUTDOWN;
	l_readReg |= mode;

	return LTC2942_setRegister(handle, LTC2942_REG_CONTROL, l_readReg);
}

/**
 * @brief      Configure coulomb counter prescaling factor M between 1 and 128.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[in]  uint8_t psc - new prescaler value (one of LTC2942_PSCM_XXX values).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool LTC2942_setPrescaler(LTC2942_t *handle, uint8_t psc)
{
	// Read CONTROL register, clear prescaler M bits and configure new value
	uint8_t l_readReg = 0;

	if(!LTC2942_getRegister(handle, LTC2942_REG_CONTROL, &l_readReg)) {
		return false;
	}

	l_readReg &= LTC2942_CTL_PSCM_MSK;
	l_readReg |= psc;

	return LTC2942_setRegister(handle, LTC2942_REG_CONTROL, l_readReg);
}

/**
 * @brief      Configure the AL/CC pin.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[in]  uint8_t mode - new pin configuration (one of LTC2942_ALCC_XXX values).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool LTC2942_setALCCMode(LTC2942_t *handle, uint8_t mode)
{
	// Read CONTROL register, clear AL/CC bits and configure new value
	uint8_t l_readReg = 0;

	if(!LTC2942_getRegister(handle, LTC2942_REG_CONTROL, &l_readReg)) {
		return false;
	}

	l_readReg &= LTC2942_CTL_ALCC_MSK;
	l_readReg |= mode;

	return LTC2942_setRegister(handle, LTC2942_REG_CONTROL, l_readReg);
}


/**
 * @brief      Configure state of the analog section of the chip.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[in]  uint8_t state - new analog section state (one of LTC2942_AN_XXX values).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool LTC2942_setAnalog(LTC2942_t *handle, uint8_t state) {
	// Read CONTROL register value
	uint8_t l_readReg = 0;

	if(!LTC2942_getRegister(handle, LTC2942_REG_CONTROL, &l_readReg)) {
		return false;
	}


	// Set new state of SHUTDOWN bit in CONTROL register B[0]
	if (state == LTC2942_AN_DISABLED) {
		l_readReg |= LTC2942_CTL_SHUTDOWN;
	} else {
		l_readReg &= ~LTC2942_CTL_SHUTDOWN;
	}

	// Write new CONTROL register value
	return LTC2942_setRegister(handle, LTC2942_REG_CONTROL, l_readReg);
}

/**
 * @brief      Read accumulated charge value.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[out] uint16_t *ac - Accumulated charge value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool LTC2942_getAC(LTC2942_t *handle, uint16_t *ac)
{
	if (!handle || !ac) {
		return false;
	}

	bool l_result = false;
	uint16_t l_dataRead = 0;

	uint8_t reg = LTC2942_REG_AC_H;

	l_result = LTC2942_i2cTransfer(handle, &reg, 1, (uint8_t*)&l_dataRead, sizeof(l_dataRead));

	if(l_result) {
		*ac = LTC2942_swapUint16(l_dataRead);
	}

	return l_result;
}


/* Private functions. */

/**
 * @brief      Read register.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[in]  uint8_t cmd - Target command.
 * @param[out] uint8_t *reg - Pointer to save register value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool LTC2942_getRegister(LTC2942_t *handle, uint8_t cmd, uint8_t *reg)
{
    return LTC2942_i2cTransfer(handle, &cmd, 1, reg, 1);
}

/**
 * @brief      Write register.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[in]  uint8_t cmd - Target command.
 * @param[in]  uint8_t reg - Register value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool LTC2942_setRegister(LTC2942_t *handle, uint8_t cmd, uint8_t reg)
{
    uint8_t l_cmdWrite[2] = {cmd, reg};

    return LTC2942_i2cTransfer(handle, l_cmdWrite, 2, 0, 0);
}

/**
 * @brief      Write and read data from I2C.
 * @param[in]  LTC2942_t *handle - Pointer to instance of LTC2942.
 * @param[in]  const uint8_t *pDataTx - Pointer to transmit data buffer.
 * @param[in]  uint16_t sizeTx - Amount of data to be sent (in bytes).
 * @param[out] uint8_t *pDataRx - Pointer to receive data buffer.
 * @param[in]  uint16_t sizeRx - amount of data to be received (in bytes).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool LTC2942_i2cTransfer(LTC2942_t *handle, const uint8_t *pDataTx, uint16_t sizeTx,
                                uint8_t *pDataRx, uint16_t sizeRx)
{
    return SeI2c_transfer(handle->m_hi2c, handle->m_address, pDataTx, sizeTx, pDataRx, sizeRx);
}

/**
 * @brief      Swap bytes in a 16-bit variable.
 * @param[in]  uint16_t value - 16-bit value in which you need to change bytes in places.
 * @retval     uint16_t - 16-bit value with changed bytes by places.
 */
static uint16_t LTC2942_swapUint16(uint16_t value)
{
    return ((value >> 8) & 0x00FF) | ((value & 0x00FF) << 8);
}
