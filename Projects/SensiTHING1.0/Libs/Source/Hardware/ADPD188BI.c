/***************************************************************************//**
 * @file   ADPD188BI.c
 * @brief  Driver for Analog Devices ADPD188BI - Integrated optical module for smoke detection.
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

#include "ADPD188BI.h"


/* Private defines. */

/* Device ID. */
#define ADPD188BI_I2CS_ID               0xC8

/* Maximum number of bytes to send. Used for send buffer. */
#define ADPD188BI_TX_PACKET_MAX_SIZE      2

/* ADPD188BI communication commands. */
#define ADPD188BI_WRITE_REG             0x01
#define ADPD188BI_READ_REG              0x00


/* Private function prototypes. */
static bool ADPD188BI_setIdleMode(ADPD188BI_t *handle);
static bool ADPD188BI_setBit(ADPD188BI_t *handle, uint8_t reg, uint8_t bit, bool value);
static bool ADPD188BI_write(ADPD188BI_t *handle, uint8_t adr, const uint8_t *pData, uint16_t size);
static bool ADPD188BI_read(ADPD188BI_t *handle, uint8_t adr, uint8_t *pData, uint16_t size);
static uint16_t ADPD188BI_swapUint16(uint16_t value);


/* Public functions. */

/**
 * @brief      Initialize ADPD188BI.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  SeI2c_t *hi2c - Pointer to the I2C with which ADPD188BI works.
 * @param[in]  uint8_t address - Chip address.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_init(ADPD188BI_t *handle, SeI2c_t *hi2c, uint8_t address)
{
    if (!handle || !hi2c) {
        return false;
    }

    handle->m_address = address;
    handle->m_hi2c = hi2c;

    if(!ADPD188BI_softReset(handle)) {
        return false;
    }

    // Test chip
    uint16_t l_i2csId = 0;
    ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_I2CS_ID, &l_i2csId);
    return(l_i2csId == ADPD188BI_I2CS_ID);
}

/**
 * @brief      Software reset.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_softReset(ADPD188BI_t *handle)
{
    if (!handle) {
        return false;
    }

    return ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_SW_RESET, 0x0001);
}

/**
 * @brief      Set ADPD188BI operating mode.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_stateMachineMode_t mode - Operation mode.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_setOperationMode(ADPD188BI_t *handle, ADPD188BI_stateMachineMode_t mode)
{
    if (!handle) {
        return false;
    }

    if (mode == ADPD188BI_STANDBY) {
        ADPD188BI_clearFifo(handle);
        return ADPD188BI_setIdleMode(handle);
    }

    // ADPD188BI_PROGRAM or ADPD188BI_NORMAL
    //  Enable (if need) the 32 kHz clock by setting the CLK32K_EN
    if(!ADPD188BI_setBit(handle, ADPD188BI_REG_SAMPLE_CLK, 7, true)) {
        return false;
    }
    return ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_MODE, mode);
}

/**
 * @brief      LEDs configuration.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_slot_t slot - Target time slot.
 * @param[in]  ADPD188BI_ledConfig_t conf - LEDs configuration.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_setLedConfig(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_ledConfig_t conf)
{
    if (!handle) {
        return false;
    }

    uint16_t l_regValue = 0;
    const uint16_t l_clearMaskA  = 0x0003;
    const uint16_t l_clearMaskB  = 0x000C;

    if(!ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_PD_LED_SELECT, &l_regValue)) {
        return false;
    }

    if(slot == ADPD188BI_SLOT_A) {
        l_regValue &= ~(l_clearMaskA);
        l_regValue |= (conf << 0);
    }
    else if(slot == ADPD188BI_SLOT_B) {
        l_regValue &= ~(l_clearMaskB);
        l_regValue |= (conf << 2);
    }

    return ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_PD_LED_SELECT, l_regValue);

}

/**
 * @brief      Photodiods configuration.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_slot_t slot - Target time slot.
 * @param[in]  ADPD188BI_pdConfig_t conf - Photodiods configuration.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_setPdConfig(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_pdConfig_t conf)
{
    if (!handle) {
        return false;
    }

    uint16_t l_regValue = 0;
    const uint16_t l_clearMaskA  = 0x00F0;
    const uint16_t l_clearMaskB  = 0x0F00;

    if(!ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_PD_LED_SELECT, &l_regValue)) {
        return false;
    }

    if(slot == ADPD188BI_SLOT_A) {
        l_regValue &= ~(l_clearMaskA);
        l_regValue |= (conf << 4);
    }
    else if(slot == ADPD188BI_SLOT_B) {
        l_regValue &= ~(l_clearMaskB);
        l_regValue |= (conf << 8);
    }

    return ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_PD_LED_SELECT, l_regValue);
}

/**
 * @brief      GPIO configuration.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_gpio_t pin - Target GPIO pin.
 * @param[in]  ADPD188BI_gpioConfig_t conf - Pin configuration.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_setGpioConfig(ADPD188BI_t *handle, ADPD188BI_gpio_t pin, ADPD188BI_gpioConfig_t conf)
{
    if (!handle) {
        return false;
    }

    uint16_t l_regValue = 0;
    const uint16_t l_clearMaskA  = 0x001F;
    const uint16_t l_clearMaskB  = 0x1F00;

    if(conf == ADPD188BI_GPIO0_DISABLE) {
         return ADPD188BI_setBit(handle, ADPD188BI_REG_GPIO_DRV, 2, false);
    }

    if(!ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_GPIO_CTRL, &l_regValue)) {
        return false;
    }

    if(pin == ADPD188BI_GPIO0) {
        l_regValue &= ~(l_clearMaskA);
        l_regValue |= (conf << 0);
    }
    else if(pin == ADPD188BI_GPIO1) {
        l_regValue &= ~(l_clearMaskB);
        l_regValue |= (conf << 8);
    }

    if(!ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_GPIO_CTRL, l_regValue)) {
        return false;
    }

    if(pin == ADPD188BI_GPIO0) {
        return ADPD188BI_setBit(handle, ADPD188BI_REG_GPIO_DRV, 2, true);
    }

    return true;
}

/**
 * @brief      Set slot FIFO configuration.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_slot_t slot - Target time slot.
 * @param[in]  ADPD188BI_slotFifoConfig_t conf - Slot FIFO configuration.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_setSlotsFifoConfig(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_slotFifoConfig_t conf)
{
    if (!handle) {
        return false;
    }

    uint16_t l_regValue = 0;
    const uint16_t l_clearMaskA  = 0x001C;
    const uint16_t l_clearMaskB  = 0x01C0;

    if(!ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_SLOT_EN, &l_regValue)) {
        return false;
    }

    if(slot == ADPD188BI_SLOT_A) {
        l_regValue &= ~(l_clearMaskA);
        l_regValue |= (conf << 2);
    }
    else if(slot == ADPD188BI_SLOT_B) {
        l_regValue &= ~(l_clearMaskB);
        l_regValue |= (conf << 6);
    }

    return ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_SLOT_EN, l_regValue);
}

/**
 * @brief      Enable/disable time slot.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_slot_t slot - Target time slot.
 * @param[in]  bool state - Enable or disable time slot.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_enableSlots(ADPD188BI_t *handle, ADPD188BI_slot_t slot, bool state)
{
    if (!handle) {
        return false;
    }

    uint16_t l_regValue = 0;

    if(!ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_SLOT_EN, &l_regValue)) {
        return false;
    }

    if(slot == ADPD188BI_SLOT_A) {
        l_regValue &= ~(1 << 0);
        l_regValue |= (state << 0);
    }
    else if(slot == ADPD188BI_SLOT_B) {
        l_regValue &= ~(1 << 5);
        l_regValue |= (state << 5);
    }

    return ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_SLOT_EN, l_regValue);
}

/**
 * @brief      Set averaging factor.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_slot_t slot - Target time slot.
 * @param[in]  ADPD188BI_averageFactor_t avg - The averaging factor.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_setAdcAvgFactor(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_averageFactor_t avg)
{
    if (!handle) {
        return false;
    }

    uint16_t l_regValue = 0;
    const uint16_t l_clearMaskA = 0x0070;
    const uint16_t l_clearMaskB = 0x0700;

    if(!ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_NUM_AVG, &l_regValue)) {
        return false;
    }

    if(slot == ADPD188BI_SLOT_A) {
        l_regValue &= ~(l_clearMaskA);
        l_regValue |= (avg << 4);
    }
    else if(slot == ADPD188BI_SLOT_B) {
        l_regValue &= ~(l_clearMaskB);
        l_regValue |= (avg << 8);
    }

    return ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_NUM_AVG, l_regValue);
}

/**
 * @brief      Enable/disable FIFO interrupt.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  uint16_t threshold - FIFO length threshold in words. If threshold == 0 interrupt disable,
 *             otherwise interrupt enable.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_enableFiFoInterrupts(ADPD188BI_t *handle, uint16_t threshold)
{
    if (!handle) {
        return false;
    }

    bool l_state = (threshold != 0);

    if(l_state) {
        // Enable interrupt.
        if(!ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_FIFO_THRESH, (threshold << 8))) {
            return false;
        }
    }

    return ADPD188BI_setBit(handle, ADPD188BI_REG_INT_MASK, 8, !l_state);
}

/**
 * @brief      Read measurement data from register.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_slot_t slot - Target time slot.
 * @param[in]  ADPD188BI_channel_t channel - Target channel.
 * @param[out] uint16_t *value - Pointer to save the read value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_readData16FromRegister(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_channel_t channel, uint16_t *value)
{
    if (!handle || !value) {
        return false;
    }

    // Register address from 0x64 to 0x6B.
    uint8_t l_reg = ADPD188BI_REG_SLOTA_PD1_16BIT + channel + (4 * slot);

    if(!ADPD188BI_setBit(handle, ADPD188BI_REG_DATA_ACCESS_CTL, 1 + slot, true)) {
        return false;
    }

    if(!ADPD188BI_getRegisterValue(handle, (ADPD188BI_registers_t)l_reg, value)) {
        return false;
    }

    return ADPD188BI_setBit(handle, ADPD188BI_REG_DATA_ACCESS_CTL, 1 + slot, false);
}

/**
 * @brief      Get the number of bytes in FIFO.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[out] uint16_t *bytes - Pointer to save the read bytes.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_getFifoBytes(ADPD188BI_t *handle, uint16_t *bytes)
{
    if (!handle || !bytes) {
        return false;
    }

    uint16_t l_regValue = 0;

    if(!ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_STATUS, &l_regValue)) {
        return false;
    }

    *bytes = (l_regValue >> 8);

    return true;
}

/**
 * @brief      Read the data from FIFO buffer in 16-bit format..
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[out  uint16_t *buffer - Pointer to buffer for save the read samples.
 * @param[in]  uint16_t buffSize - Buffer size, in bytes.
 * @param[in]  uint16_t samples - Number of samples to read, in words.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_readData16FromFifo(ADPD188BI_t *handle, uint16_t *buffer, uint16_t buffSize, uint16_t samples)
{
    if (!handle || !buffer || (ADPD188BI_FIFO_BUF_MAX_SIZE < buffSize) || (buffSize < (samples * 2))) {
        return false;
    }

    if(!ADPD188BI_read(handle, ADPD188BI_REG_FIFO_ACCESS, handle->m_buffer, samples * 2)) {
        return false;
    }

    uint16_t * l_pBuff = (uint16_t*)handle->m_buffer;

    for(int i = 0; i < samples; i++) {
        l_pBuff[i] = ADPD188BI_swapUint16(l_pBuff[i]);
    }

    memcpy(buffer, handle->m_buffer, samples*2);

    return true;
}

/**
 * @brief      Clear hardware FIFO buffer.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_clearFifo(ADPD188BI_t *handle)
{
    if (!handle) {
        return false;
    }

    uint16_t l_regValue = 0;

    if(!ADPD188BI_getRegisterValue(handle, ADPD188BI_REG_MODE, &l_regValue)) {
        return false;
    }

    if(!ADPD188BI_setOperationMode(handle, ADPD188BI_PROGRAM)) {
        return false;
    }
    if(!ADPD188BI_setBit(handle, ADPD188BI_REG_STATUS, 15, true)) {
        return false;
    }

    return ADPD188BI_setOperationMode(handle, l_regValue);

}

/**
 * @brief      Set register value.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_registers_t reg - Register address.
 * @param[in]  uint16_t value - New register value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_setRegisterValue(ADPD188BI_t *handle, ADPD188BI_registers_t reg, uint16_t value)
{
    if (!handle) {
        return false;
    }

    uint16_t l_value = ADPD188BI_swapUint16(value);

    return ADPD188BI_write(handle, reg, (uint8_t*)(&l_value), 2);
}

/**
 * @brief      Get register value.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_registers_t reg - Register address.
 * @param[out] uint16_t *value - Pointer to save the read value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADPD188BI_getRegisterValue(ADPD188BI_t *handle, ADPD188BI_registers_t reg, uint16_t *value)
{
    if (!handle) {
        return false;
    }

    uint16_t l_value = 0;

    if(!ADPD188BI_read(handle, reg, (uint8_t*)(&l_value), 2)) {
        return false;
    }

    *value = ADPD188BI_swapUint16(l_value);

    return true;
}


/* Private functions. */

/**
 * @brief      Set device to Idle mode.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADPD188BI_setIdleMode(ADPD188BI_t *handle)
{
    if (!handle) {
        return false;
    }

    // Enter to program mode
    if(!ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_MODE, ADPD188BI_PROGRAM)) {
        return false;
    }

    // Clear interrupts
    if(!ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_STATUS, 0x00FF)) {
        return false;
    }

    // Enter to standby mode
    if(!ADPD188BI_setRegisterValue(handle, ADPD188BI_REG_MODE, ADPD188BI_STANDBY)) {
        return false;
    }

    //  Stop the 32 kHz clock by resetting the CLK32K_EN
    return ADPD188BI_setBit(handle, ADPD188BI_REG_SAMPLE_CLK, 7, false);
}

/**
 * @brief      Set or clear bit in register.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  uint8_t reg - Register address.
 * @param[in]  uint8_t bit - bit position to be write.
 * @param[in]  bool value - bit value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADPD188BI_setBit(ADPD188BI_t *handle, uint8_t reg, uint8_t bit, bool value)
{
    uint16_t l_currentState = 0;

    if(!ADPD188BI_getRegisterValue(handle, reg, &l_currentState)) {
        return false;
    }

    if(value) {
       l_currentState |= (1 << bit);
    } else {
       l_currentState &= ~(1 << bit);
    }

    return ADPD188BI_setRegisterValue(handle, reg, l_currentState);
}

/**
 * @brief      Write data.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  uint8_t adr - Address of the register.
 * @param[in]  const uint8_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be write.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADPD188BI_write(ADPD188BI_t *handle, uint8_t adr, const uint8_t *pData, uint16_t size)
{
//    bool l_result = false;
//    uint8_t l_header =  (adr << 1) | ADPD188BI_WRITE_REG;

//    if(SeSpi_transmit(handle->m_hspi, &l_header, 1) && SeSpi_transmit(handle->m_hspi, pData, size)) {
//        l_result = true;
//    }

//    return l_result;


    if(size > ADPD188BI_TX_PACKET_MAX_SIZE) {
	   return false;
	}

	uint8_t l_transmit[ADPD188BI_TX_PACKET_MAX_SIZE + 1];

	l_transmit[0] = adr;

	memcpy(&l_transmit[1], pData, size);

	return SeI2c_transmit(handle->m_hi2c, handle->m_address, l_transmit, size + 1);

}

/**
 * @brief      Read data.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI
 * @param[in]  uint8_t adr - Address of the register.
 * @param[out] uint16_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be read.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADPD188BI_read(ADPD188BI_t *handle, uint8_t adr, uint8_t *pData, uint16_t size)
{
//    bool l_result = false;
//    uint8_t l_header =  (adr << 1) | ADPD188BI_READ_REG;

//    if(SeSpi_transmit(handle->m_hspi, &l_header, 1) && SeSpi_receive(handle->m_hspi, pData, size)) {
//        l_result = true;
//    }

//    return l_result;

    return SeI2c_transfer(handle->m_hi2c, handle->m_address, &adr, 1, pData, size);
}

/**
 * @brief      Swap bytes in a 16-bit variable.
 * @param[in]  uint16_t value - 16-bit value in which you need to change bytes in places.
 * @retval     uint16_t - 16-bit value with changed bytes by places.
 */
static uint16_t ADPD188BI_swapUint16(uint16_t value)
{
    return ((value >> 8) & 0x00FF) | ((value & 0x00FF) << 8);
}

