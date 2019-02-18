/***************************************************************************//**
 * @file   AD5592R.c
 * @brief  Driver for Analog Devices AD5592R - configurable ADC/DAC/GPIO.
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

#include "AD5592R.h"


/* Private defines. */

/* Registers bits. */
#define AD5592R_GPIO_READBACK_EN    (1 << 10)
#define AD5592R_LDAC_READBACK_EN    (1 << 6)
#define AD5592R_DAC_READBACK_EN     (3 << 3)
#define AD5592R_DAC_UPD_COMMAND     (2)
#define AD5592R_DAC_WRITE           (1 << 15)

/* AD5592R Reset settings. */
#define AD5592R_RESET_KEY           0x5AC

/* AD5592R pin numbers. */
#define AD5592R_PIN_NBR             8


/* Private function prototypes. */
static bool AD5592R_spiWrite(AD5592R_t *handle, const uint16_t *pData, uint16_t size);
static bool AD5592R_spiRead(AD5592R_t *handle, uint16_t *pData, uint16_t size);
static bool AD5592R_setBit(AD5592R_t *handle, uint8_t reg, uint8_t bit, bool value);
static uint16_t AD5592R_swapUint16(uint16_t value);
static void AD5592R_setDefaultValues(AD5592R_t *handle);


/* Public functions. */

/**
 * @brief      Initialize AD5592R.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  const SeGpio_pin_t *cs - Chip select pin.
 * @param[in]  SeSpi_t *hspi - Pointer to the SPI with which AD5592R works.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_init(AD5592R_t *handle, const SeGpio_pin_t *cs, SeSpi_t *hspi)
{
    if (!handle || !hspi || !cs) {
        return false;
    }

    handle->m_hspi = hspi;
    handle->m_cs = cs;

    if(!AD5592R_softReset(handle)) {
        return false;
    }

    // Test IC
    uint16_t l_nop = 0;
    uint16_t l_pullDown = 0;

    AD5592R_getRegisterValue(handle, &l_nop, AD5592R_REG_NOP);
    AD5592R_getRegisterValue(handle, &l_pullDown, AD5592R_REG_PULLDOWN);

    if(l_nop != 0 || l_pullDown != 0xFF) {
        return false;
    }

    return true;
}

/**
 * @brief      Software reset.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_softReset(AD5592R_t *handle)
{
    if(!handle) {
        return false;
    }

    AD5592R_setDefaultValues(handle);

    if(!AD5592R_setRegisterValue(handle, AD5592R_RESET_KEY, AD5592R_REG_RESET)) {
        return false;
    }

    WAIT(1);

    return true;
}

/**
 * @brief      Enable or disable power down mode state.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  bool value - New value 'true' - enable, 'false' - disable.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_powerDown(AD5592R_t *handle, bool value)
{
    if(!handle) {
        return false;
    }

    return AD5592R_setBit(handle, AD5592R_REG_PD, 10, value);
}

/**
 * @brief      Enable or disable internal reference.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  bool value - New value 'true' - enable, 'false' - disable.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_setIntReference(AD5592R_t *handle, bool value)
{
    if(!handle) {
        return false;
    }

    return AD5592R_setBit(handle, AD5592R_REG_PD, 9, value);
}

/**
 * @brief      Configuration I/O pin.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  AD5592R_pinMode_t mode - Pin mode.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_confPin(AD5592R_t *handle, uint8_t pin, AD5592R_pinMode_t mode)
{
    if(!handle || pin >= AD5592R_PIN_NBR) {
        return false;
    }

    bool l_result = false;

    switch (mode) {
        case AD5592R_PIN_MODE_PULLDOWN:
            l_result = AD5592R_setBit(handle, AD5592R_REG_PULLDOWN, pin, true);
            break;

        case AD5592R_PIN_MODE_DAC:
            l_result = AD5592R_setBit(handle, AD5592R_REG_DAC_EN, pin, true);
            break;

        case AD5592R_PIN_MODE_ADC:
            l_result = AD5592R_setBit(handle, AD5592R_REG_ADC_EN, pin, true);
            break;

        case AD5592R_PIN_MODE_DAC_ADC:
            l_result = AD5592R_setBit(handle, AD5592R_REG_DAC_EN, pin, true);
            l_result = AD5592R_setBit(handle, AD5592R_REG_ADC_EN, pin, true);
            break;

        case AD5592R_PIN_MODE_GPIO_IN:
            l_result = AD5592R_setBit(handle, AD5592R_REG_GPIO_IN_EN, pin, true);
            break;

        case AD5592R_PIN_MODE_GPIO_OUT:
            l_result = AD5592R_setBit(handle, AD5592R_REG_GPIO_OUT_EN, pin, true);
            break;

        case AD5592R_PIN_MODE_GPIO_IN_OUT:
            l_result = AD5592R_setBit(handle, AD5592R_REG_GPIO_IN_EN, pin, true);
            l_result = AD5592R_setBit(handle, AD5592R_REG_GPIO_OUT_EN, pin, true);
            break;

        case AD5592R_PIN_MODE_OPEN_DRAIN:
            l_result = AD5592R_setBit(handle, AD5592R_REG_OPEN_DRAIN, pin, true);
            break;

        case AD5592R_PIN_MODE_TRISTATE:
            l_result = AD5592R_setBit(handle, AD5592R_REG_TRISTATE, pin, true);
            break;

        default:
            break;
    }

    return l_result;
}

/**
 * @brief      Write data to GPIO output.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  uint8_t pin - Pin position.
 * @param[in]  bool value - New pin value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_gpioWriteData(AD5592R_t *handle, uint8_t pin, bool value)
{
    if(!handle) {
        return false;
    }

    return AD5592R_setBit(handle, AD5592R_REG_GPIO_SET, pin, value);
}

/**
 * @brief      Read value on GPIO input.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  uint8_t pin - Pin position.
 * @param[out] bool *value - Pointer to save pin value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_gpioReadData(AD5592R_t *handle, uint8_t pin, bool *value)
{
    if(!handle || !value) {
        return false;
    }

    uint16_t l_currentState = 0;
    uint16_t l_readValue = 0;

    if(!AD5592R_getRegisterValue(handle, &l_currentState, AD5592R_REG_GPIO_IN_EN)) {
       return false;
    }

    uint16_t l_writeValue = (AD5592R_REG_GPIO_IN_EN << 11) |  AD5592R_GPIO_READBACK_EN | l_currentState;

    if(!AD5592R_spiWrite(handle, &l_writeValue, 1) || !AD5592R_spiRead(handle, &l_readValue, 1)) {
        return false;
    }

    *value = (l_readValue & (1 << pin));

    return true;
}

/**
 * @brief      Enable or disable ADC buffer.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  bool value - New value 'true' - enable, 'false' - disable.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_adcEnableBuffer(AD5592R_t *handle, bool value)
{
    if(!handle) {
        return false;
    }

    return AD5592R_setBit(handle, AD5592R_REG_CTRL, 8, value);
}

/**
 * @brief      Set ADC gain.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  AD5592R_gain_t gain - New ADC gain.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_adcSetGain(AD5592R_t *handle, AD5592R_gain_t gain)
{
    if(!handle) {
        return false;
    }

    if(!AD5592R_setBit(handle, AD5592R_REG_CTRL, 5, (gain == AD5592R_GAIN_2))) {
        return false;
    }

    handle->m_adcGain = gain;

    return true;
}

/**
 * @brief      Read data from ADC input.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  uint8_t pin - Pin position.
 * @param[out] uint16_t *value - Pointer to save the read value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_adcReadData(AD5592R_t *handle, uint8_t pin, uint16_t *value)
{
    if(!handle || !value) {
        return false;
    }

    uint16_t l_writeValue = (1 << (pin & 0x0007));
    uint16_t l_readValue = 0;

    if(!AD5592R_setRegisterValue(handle, l_writeValue, AD5592R_REG_ADC_SEQ)) {
        return false;
    }

    // Send any data - NOP and read result
    if(!AD5592R_spiWrite(handle, &l_writeValue, 1) || !AD5592R_spiRead(handle, &l_readValue, 1)) {
        return false;
    }

    *value = (l_readValue & 0xFFF);

    return true;
}

/**
 * @brief      Read temperature.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[out] float *value - Pointer to save the temperature in degrees Celsius.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_adcReadTemperature(AD5592R_t *handle, float *value)
{
    if(!handle || !value) {
        return false;
    }

    uint16_t l_writeValue = (1 << 8);
    uint16_t l_readValue = 0;

    if(!AD5592R_setRegisterValue(handle, l_writeValue, AD5592R_REG_ADC_SEQ)) {
        return false;
    }

    // Wait temperature conversion
    WAIT(1);

    // Send any data - NOP and read result
    if(!AD5592R_spiWrite(handle, &l_writeValue, 1) || !AD5592R_spiRead(handle, &l_readValue, 1)) {
        return false;
    }

    *value = 25.0 + (((l_readValue & 0xFFF) - (820/handle->m_adcGain)) / 2.654);

    return true;
}

/**
 * @brief      Determines how data written to an input register of a DAC is handled.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  AD5592R_ldacMode_t mode - New mode.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_dacSetMode(AD5592R_t *handle, AD5592R_ldacMode_t mode)
{
    if(!handle) {
        return false;
    }

    uint16_t l_currentState = 0;

    if(!AD5592R_getRegisterValue(handle, &l_currentState, AD5592R_REG_LDAC)) {
        return false;
    }

     // Reset mode (bits D0, D1)
     l_currentState &= ~(0x003);
     // Set new mode
     l_currentState |= mode;

     if(!AD5592R_setRegisterValue(handle, l_currentState, AD5592R_REG_LDAC)) {
         return false;
     }

     // Save mode to config
     handle->m_ldacMode = mode;

     return true;
}

/**
 * @brief      Manual update DAC output, when select mode 'AD5592R_DAC_MODE_NOT_UPD'.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_dacUpdateOutputs(AD5592R_t *handle)
{
    if(!handle) {
        return false;
    }

    return AD5592R_dacSetMode(handle, AD5592R_DAC_UPD_COMMAND);
}

/**
 * @brief      Set DAC gain.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  AD5592R_gain_t gain - New DAC gain.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_dacSetGain(AD5592R_t *handle, AD5592R_gain_t gain)
{
    return AD5592R_setBit(handle, AD5592R_REG_CTRL, 4, (gain == AD5592R_GAIN_2));
}

/**
 * @brief      Write data to DAC output.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  uint8_t pin - Pin position.
 * @param[in]  uint16_t value - New DAC value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_dacWriteData(AD5592R_t *handle, uint8_t pin, uint16_t value)
{
    if(!handle) {
        return false;
    }

    uint16_t l_writeValue = AD5592R_DAC_WRITE | (((pin & 0x0007) << 12) | (value & 0x0FFF));
    return AD5592R_spiWrite(handle, &l_writeValue, 1);
}

/**
 * @brief      Read data from DAC output.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  uint8_t pin - Pin position.
 * @param[out] uint16_t *value - Pointer to save the read value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_dacReadData(AD5592R_t *handle, uint8_t pin, uint16_t *value)
{
    if(!handle || !value) {
        return false;
    }

    uint16_t l_writeValue = (AD5592R_REG_DAC_READBACK << 11) | AD5592R_DAC_READBACK_EN | (pin & 0x0007);
    uint16_t l_readValue = 0;

    if(!AD5592R_spiWrite(handle, &l_writeValue, 1) || !AD5592R_spiRead(handle, &l_readValue, 1)) {
        return false;
    }

    if((l_readValue & AD5592R_DAC_READBACK_EN) && ((l_readValue >> 12) & 0x7) == pin) {
        *value = (l_readValue & 0xFFF);
    } else {
        return false;
    }

    return true;
}

/**
 * @brief      Set register value.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  uint16_t value - New register value.
 * @param[in]  AD5592R_registers_t reg - Register address.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_setRegisterValue(AD5592R_t *handle, uint16_t value, AD5592R_registers_t reg)
{
    if(!handle) {
        return false;
    }

    uint16_t l_writeValue = ((reg << 11) | (value & 0x7FF));
    return AD5592R_spiWrite(handle, &l_writeValue, 1);
}

/**
 * @brief      Read register value.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[out] uint16_t *value - Pointer to save the read value.
 * @param[in]  AD5592R_registers_t reg - Register address.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool AD5592R_getRegisterValue(AD5592R_t *handle, uint16_t *value, AD5592R_registers_t reg)
{
    if(!handle || !value) {
        return false;
    }

    // Create command and restore LDAC mode
    uint16_t l_writeValue = (AD5592R_REG_LDAC << 11) |  AD5592R_LDAC_READBACK_EN | (reg << 2) | handle->m_ldacMode;

    if(!AD5592R_spiWrite(handle, &l_writeValue, 1) || !AD5592R_spiRead(handle, value, 1)) {
        return false;
    }

    return true;
}


/* Private functions. */

/**
 * @brief      Set or clear bit in register.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  uint8_t reg - Register address.
 * @param[in]  uint8_t bit - bit position to be write.
 * @param[in]  bool value - bit value.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool AD5592R_setBit(AD5592R_t *handle, uint8_t reg, uint8_t bit, bool value)
{
    uint16_t l_currentState = 0;

    if(!AD5592R_getRegisterValue(handle, &l_currentState, reg)) {
        return false;
    }

    l_currentState &= 0x07FF;

    if(value) {
        l_currentState |= (1 << bit);
    } else {
        l_currentState &= ~(1 << bit);
    }

    return AD5592R_setRegisterValue(handle, l_currentState, reg);
}

/**
 * @brief      Write data to SPI.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[in]  const uint16_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be write.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool AD5592R_spiWrite(AD5592R_t *handle, const uint16_t *pData, uint16_t size)
{
    bool l_result = false;
    uint16_t l_swapBuff[4];

    for(int i = 0; i < size; i++) {
        l_swapBuff[i] = AD5592R_swapUint16(pData[i]);
    }

    SeGpio_setOutPin(handle->m_cs, false);
    l_result = SeSpi_transmit(handle->m_hspi, (uint8_t*)l_swapBuff, size*2);
    SeGpio_setOutPin(handle->m_cs, true);

    return l_result;
}

/**
 * @brief      Read data from SPI.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @param[out] uint16_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be read.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool AD5592R_spiRead(AD5592R_t *handle, uint16_t* pData, uint16_t size)
{
    bool l_result = false;
    uint16_t l_swapBuff[4];

    SeGpio_setOutPin(handle->m_cs, false);
    l_result = SeSpi_receive(handle->m_hspi, (uint8_t*)l_swapBuff, size*2);
    SeGpio_setOutPin(handle->m_cs, true);

    if(l_result){
        for(int i = 0; i < size; i++) {
            l_swapBuff[i] = AD5592R_swapUint16(l_swapBuff[i]);
        }
        memcpy(pData, l_swapBuff, size*2);
    }

    return l_result;
}

/**
 * @brief      Swap bytes in a 16-bit variable.
 * @param[in]  uint16_t value - 16-bit value in which you need to change bytes in places.
 * @retval     uint16_t - 16-bit value with changed bytes by places.
 */
static uint16_t AD5592R_swapUint16(uint16_t value)
{
    return (uint16_t)((value >> 8) & 0x00FF) | ((value & 0x00FF) << 8);
}

/**
 * @brief      Set default values in working structure.
 * @param[in]  AD5592R_t *handle - Pointer to instance of AD5592R.
 * @retval     None.
 */
static void AD5592R_setDefaultValues(AD5592R_t *handle)
{
    handle->m_ldacMode = AD5592R_DAC_MODE_IMMEDIATELY_UPD;
    handle->m_adcGain = AD5592R_GAIN_1;
}


