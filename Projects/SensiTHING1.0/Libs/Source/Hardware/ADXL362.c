/***************************************************************************//**
 * @file   ADXL362.h
 * @brief  Driver for Analog Devices ADXL362 - Micropower 3-axis accelerometer
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

#include "ADXL362.h"


/* Private defines. */

/* ADXL362 communication commands. */
#define ADXL362_WRITE_REG                   0x0A
#define ADXL362_READ_REG                    0x0B
#define ADXL362_WRITE_FIFO                  0x0D

/* ADXL362 reset settings. */
#define ADXL362_RESET_KEY                   0x52

/* ADXL362 device information. */
#define ADXL362_DEVICE_AD                   0xAD
#define ADXL362_DEVICE_MST                  0x1D
#define ADXL362_PART_ID                     0xF2

/* ADXL362_REG_ACT_INACT_CTL definitions. */
#define ADXL362_ACT_INACT_CTL_LINKLOOP(x)   (((x) & 0x3) << 4)
#define ADXL362_ACT_INACT_CTL_INACT_REF     (1 << 3)
#define ADXL362_ACT_INACT_CTL_INACT_EN      (1 << 2)
#define ADXL362_ACT_INACT_CTL_ACT_REF       (1 << 1)
#define ADXL362_ACT_INACT_CTL_ACT_EN        (1 << 0)

/* ADXL362_REG_FIFO_CTL definitions. */
#define ADXL362_FIFO_CTL_AH                 (1 << 3)
#define ADXL362_FIFO_CTL_FIFO_TEMP          (1 << 2)
#define ADXL362_FIFO_CTL_FIFO_MODE(x)       (((x) & 0x3) << 0)

/* ADXL362_REG_FILTER_CTL definitions. */
#define ADXL362_FILTER_CTL_RANGE(x)         (((x) & 0x3) << 6)
#define ADXL362_FILTER_CTL_RES              (1 << 5)
#define ADXL362_FILTER_CTL_HALF_BW          (1 << 4)
#define ADXL362_FILTER_CTL_EXT_SAMPLE       (1 << 3)
#define ADXL362_FILTER_CTL_ODR(x)           (((x) & 0x7) << 0)

/* ADXL362_REG_POWER_CTL definitions. */
#define ADXL362_POWER_CTL_RES               (1 << 7)
#define ADXL362_POWER_CTL_EXT_CLK           (1 << 6)
#define ADXL362_POWER_CTL_LOW_NOISE(x)      (((x) & 0x3) << 4)
#define ADXL362_POWER_CTL_WAKEUP            (1 << 3)
#define ADXL362_POWER_CTL_AUTOSLEEP         (1 << 2)
#define ADXL362_POWER_CTL_MEASURE(x)        (((x) & 0x3) << 0)

/* ADXL362_POWER_CTL_MEASURE(x) options. */
#define ADXL362_MEASURE_STANDBY             0
#define ADXL362_MEASURE_ON                  2

/* ADXL362_POWER_CTL_LOW_NOISE(x) options. */
#define ADXL362_NOISE_MODE_NORMAL           0
#define ADXL362_NOISE_MODE_LOW              1
#define ADXL362_NOISE_MODE_ULTRALOW         2

/* ADXL362_ACT_INACT_CTL_LINKLOOP(x) options. */
#define ADXL362_MODE_DEFAULT                0
#define ADXL362_MODE_LINK                   1
#define ADXL362_MODE_LOOP                   3


/* Private function prototypes. */
static bool ADXL362_spiWrite(ADXL362_t *handle, uint8_t adr, const uint8_t *pData, uint16_t size);
static bool ADXL362_spiRead(ADXL362_t *handle, uint8_t adr, uint8_t *pData, uint16_t size);
static bool ADXL362_spiReadFifo(ADXL362_t *handle, uint8_t *pData, uint16_t size);


/* Public functions. */

/**
 * @brief      Initialize ADXL362.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  const SeGpio_pin_t cs - Chip select pin.
 * @param[in]  SeSpi_t *hspi - Pointer to the SPI with which ADXL362 works.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_init(ADXL362_t *handle, const SeGpio_pin_t *cs, SeSpi_t *hspi)
{
    if (!handle || !cs || !hspi) {
        return false;
    }

    handle->m_hspi = hspi;
    handle->m_cs = cs;
    // Measurement Range: +/- 2g (reset default)
    handle->m_range = 2;

    // Write to SOFT RESET, "R".
    if(!ADXL362_setRegisterValue(handle, ADXL362_RESET_KEY, ADXL362_REG_SOFT_RESET)) {
        return false;
    }

    WAIT(10);

    return ADXL362_check(handle);
}

/**
 * @brief      Checking Component presence reading the device ids.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @retval     bool - Checking status 'true' if device correct, 'false' - otherwise.
 */
bool ADXL362_check(ADXL362_t *handle)
{
    if (!handle) {
        return false;
    }

    uint8_t l_ad = 0;
    uint8_t l_mst = 0;
    uint8_t l_partId = 0;

    ADXL362_getRegisterValue(handle, &l_ad, ADXL362_REG_DEVID_AD);
    ADXL362_getRegisterValue(handle, &l_mst, ADXL362_REG_DEVID_MST);
    ADXL362_getRegisterValue(handle, &l_partId, ADXL362_REG_PARTID);

    if (l_ad != ADXL362_DEVICE_AD || l_mst != ADXL362_DEVICE_MST || l_partId != ADXL362_PART_ID) {
        return false;
    }

    return true;
}

/**
 * @brief      Writes data into a register.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  uint8_t regValue - Data value to write.
 * @param[in]  ADXL362_registers_t reg - Address of the register.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setRegisterValue(ADXL362_t *handle, uint8_t regValue, ADXL362_registers_t reg)
{
    if (!handle) {
        return false;
    }

    return ADXL362_spiWrite(handle, reg, &regValue, 1);
}

/**
 * @brief      Read data from register.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[out] uint8_t *pReadData - The read value is stored in this buffer.
 * @param[in]  ADXL362_registers_t reg - Address of the register.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_getRegisterValue(ADXL362_t *handle, uint8_t *pReadData, ADXL362_registers_t reg)
{
    if (!handle || !pReadData) {
        return false;
    }

    return ADXL362_spiRead(handle, reg, pReadData, 1);
}

/**
 * @brief      Get status.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[out] ADXL362_statusReg_t *status - Pointer to status register structure.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_getStatus(ADXL362_t *handle, ADXL362_statusReg_t *status)
{
    if (!handle || !status) {
        return false;
    }

    return ADXL362_getRegisterValue(handle, &status->reg, ADXL362_REG_STATUS);
}

/**
 * @brief      Performs a burst write of a specified number of registers.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  const uint8_t *pWriteData - Pointer to data to be write.
 * @param[in]  ADXL362_registers_t reg - The start address of the burst write.
 * @param[in]  uint16_t bytesNumber - Number of bytes to write.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_burstWite(ADXL362_t *handle, const uint8_t *pWriteData, ADXL362_registers_t reg, uint16_t bytesNumber)
{
    if (!handle || !pWriteData || !bytesNumber) {
        return false;
    }

    return ADXL362_spiWrite(handle, reg, pWriteData, bytesNumber);
}

/**
 * @brief      Performs a burst read of a specified number of registers.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[out] uint8_t *pReadData - The read values are stored in this buffer.
 * @param[in]  ADXL362_registers_t reg - The start address of the burst read.
 * @param[in]  uint16_t bytesNumber - Number of bytes to read.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_burstRead(ADXL362_t *handle, uint8_t *pReadData, ADXL362_registers_t reg, uint16_t bytesNumber)
{
    if (!handle || !pReadData || !bytesNumber) {
        return false;
    }

    return ADXL362_spiRead(handle, reg, pReadData, bytesNumber);
}

/**
 * @brief      Reads multiple bytes from the device's FIFO buffer.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  uint8_t *pBuffer - Stores the read bytes.
 * @param[in]  uint16_t bytesNumber - Number of bytes to read.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_getFifoValue(ADXL362_t *handle, uint8_t *pBuffer, uint16_t bytesNumber)
{
    if (!handle || !pBuffer || !bytesNumber) {
        return false;
    }

    return ADXL362_spiReadFifo(handle, pBuffer, bytesNumber);
}

/**
 * @brief      Resets the device via SPI communication bus.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_softwareReset(ADXL362_t *handle)
{
    if (!handle) {
        return false;
    }

    return ADXL362_setRegisterValue(handle, ADXL362_RESET_KEY, ADXL362_REG_SOFT_RESET);
}

/**
 * @brief      Places the device into standby or measure mode.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  ADXL362_powerMode_t pwrMode - Power mode.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setPowerMode(ADXL362_t *handle, ADXL362_powerMode_t pwrMode)
{
    if (!handle) {
        return false;
    }

    uint8_t l_oldPowerCtl = 0;
    uint8_t l_newPowerCtl = 0;

    if(!ADXL362_getRegisterValue(handle, &l_oldPowerCtl, ADXL362_REG_POWER_CTL)) {
        return false;
    }

    l_newPowerCtl = l_oldPowerCtl & ~ADXL362_POWER_CTL_MEASURE(0x3);
    l_newPowerCtl = l_newPowerCtl | (pwrMode * ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON));

    return ADXL362_setRegisterValue(handle, l_newPowerCtl, ADXL362_REG_POWER_CTL);
}

/**
 * @brief      Places the device into wakeup mode.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  bool wakeup - Wakeup mode: 'true' wakeup, 'false' - no wakeup.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setWakeupMode(ADXL362_t *handle, bool wakeup)
{
    if (!handle) {
        return false;
    }

    uint8_t l_oldPowerCtl = 0;
    uint8_t l_newPowerCtl = 0;

    if(!ADXL362_getRegisterValue(handle, &l_oldPowerCtl, ADXL362_REG_POWER_CTL)) {
        return false;
    }

    l_newPowerCtl = l_oldPowerCtl & ~ADXL362_POWER_CTL_MEASURE(0x8);

    if (wakeup) {
        l_newPowerCtl = l_newPowerCtl | ADXL362_POWER_CTL_WAKEUP;
    }

    return ADXL362_setRegisterValue(handle, l_newPowerCtl, ADXL362_REG_POWER_CTL);
}

/**
 * @brief      Selects the measurement range.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  ADXL362_range_t gRange - Range option.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setRange(ADXL362_t *handle, ADXL362_range_t gRange)
{
    if (!handle) {
        return false;
    }

    uint8_t l_oldFilterCtl = 0;
    uint8_t l_newFilterCtl = 0;

    if(!ADXL362_getRegisterValue(handle, &l_oldFilterCtl, ADXL362_REG_FILTER_CTL)) {
        return false;
    }

    l_newFilterCtl = l_oldFilterCtl & ~ADXL362_FILTER_CTL_RANGE(0x3);
    l_newFilterCtl = l_newFilterCtl | ADXL362_FILTER_CTL_RANGE(gRange);

    if(!ADXL362_setRegisterValue(handle, l_newFilterCtl, ADXL362_REG_FILTER_CTL)) {
        return false;
    }

    handle->m_range = (1 << gRange) * 2;

    return true;
}

/**
 * @brief      Selects the Output Data Rate of the device.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  ADXL362_outDataRate_t outRate - Output Data Rate option.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setOutputRate(ADXL362_t *handle, ADXL362_outDataRate_t outRate)
{
    if (!handle) {
        return false;
    }

    uint8_t l_oldFilterCtl = 0;
    uint8_t l_newFilterCtl = 0;

    if(!ADXL362_getRegisterValue(handle, &l_oldFilterCtl, ADXL362_REG_FILTER_CTL)) {
        return false;
    }

    l_newFilterCtl = l_oldFilterCtl & ~ADXL362_FILTER_CTL_ODR(0x7);
    l_newFilterCtl = l_newFilterCtl | ADXL362_FILTER_CTL_ODR(outRate);

    return ADXL362_setRegisterValue(handle, l_newFilterCtl, ADXL362_REG_FILTER_CTL);
}

/**
 * @brief      Reads the 3-axis raw data from the accelerometer.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[out] int16_t *x - Stores the X-axis data(as two's complement).
 * @param[out] int16_t *y - Stores the Y-axis data(as two's complement).
 * @param[out] int16_t *z - Stores the Z-axis data(as two's complement).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_getXyz(ADXL362_t *handle, int16_t *x, int16_t *y, int16_t *z)
{
    if (!handle) {
        return false;
    }

    uint8_t l_xyzValues[6] = {0, 0, 0, 0, 0, 0};

    if(!ADXL362_burstRead(handle, l_xyzValues, ADXL362_REG_XDATA_L, 6)) {
        return false;
    }

    memcpy((uint8_t*)x, &l_xyzValues[0], 2);
    memcpy((uint8_t*)y, &l_xyzValues[2], 2);
    memcpy((uint8_t*)z, &l_xyzValues[4], 2);

    return true;
}

/**
 * @brief      Reads the 3-axis raw data from the accelerometer and converts it to G.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[out] float *x - Stores the X-axis data(as two's complement).
 * @param[out] float *y - Stores the Y-axis data(as two's complement).
 * @param[out] float *z - Stores the Z-axis data(as two's complement).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_getGxyz(ADXL362_t *handle, float *x, float *y, float *z)
{
    if (!handle) {
        return false;
    }

    uint8_t l_xyzValues[6] = {0, 0, 0, 0, 0, 0};
    int16_t l_sValue = 0;

    if(!ADXL362_burstRead(handle, l_xyzValues, ADXL362_REG_XDATA_L, 6)) {
        return false;
    }

    memcpy((uint8_t*)(&l_sValue), &l_xyzValues[0], 2);
    *x = ((float)l_sValue) / (1000 / (handle->m_range / 2));

    memcpy((uint8_t*)(&l_sValue), &l_xyzValues[2], 2);
    *y = ((float)l_sValue) / (1000 / (handle->m_range / 2));

    memcpy((uint8_t*)(&l_sValue), &l_xyzValues[4], 2);
    *z = ((float)l_sValue) / (1000 / (handle->m_range / 2));

    return true;
}

/**
 * @brief      Reads the temperature of the device.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[out] float *temperature - Pointer to save value of the temperature(degrees Celsius).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_readTemperature(ADXL362_t *handle, float *temperature)
{
    if (!handle) {
        return false;
    }

    int16_t l_rawTemp = 0;

    if(!ADXL362_burstRead(handle, (uint8_t*)(&l_rawTemp), ADXL362_REG_TEMP_L, 2)) {
        return false;
    }

    *temperature = (float)l_rawTemp * 0.065;

    return true;
}

/**
 * @brief      Configures the FIFO feature.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  ADXL362_fifoMode_t mode - Mode selection.
 * @param[in]  uint16_t waterMarkLvl - Specifies the number of samples to store in the FIFO.
 * @param[in]  bool includeTemp - Temperature Data to FIFO. 'true' - temperature data is stored in the
 *             FIFO together with x-, y- and x-axis data. 'false' - temperature data is skipped.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_fifoSetup(ADXL362_t *handle, ADXL362_fifoMode_t mode, uint16_t waterMarkLvl, bool includeTemp)
{
    if (!handle) {
        return false;
    }

    uint8_t l_writeVal = 0;

    l_writeVal = ADXL362_FIFO_CTL_FIFO_MODE(mode) | (includeTemp * ADXL362_FIFO_CTL_FIFO_TEMP) | ADXL362_FIFO_CTL_AH;

    if(!ADXL362_setRegisterValue(handle, l_writeVal, ADXL362_REG_FIFO_CTL)) {
        return false;
    }

    return ADXL362_burstWite(handle, (uint8_t*)(&waterMarkLvl), ADXL362_REG_FIFO_SAMPLES, 2);
}

/**
 * @brief      Configures activity detection.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  bool referenceMode - Referenced/Absolute Activity Select. 'true' - referenced mode.
 *             'false' - absolute mode.
 * @param[in]  uint16_t threshold - 11-bit unsigned value that the adxl362 samples are compared to.
 * @param[in]  uint8_t time - 8-bit value written to the activity timer register. The amount of time (in seconds) is:
 *             time / ODR, where ODR - is the output data rate.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setupActivityDetection(ADXL362_t *handle, bool referenceMode, uint16_t threshold, uint8_t time)
{
    if (!handle) {
        return false;
    }

    uint8_t l_oldActInactReg = 0;
    uint8_t l_newActInactReg = 0;
    uint16_t l_threshold = threshold & 0x7FF;

    // Configure motion threshold and activity timer
    if(!ADXL362_burstWite(handle, (uint8_t*)(&l_threshold), ADXL362_REG_THRESH_ACT_L, 2)) {
        return false;
    }
    if(!ADXL362_setRegisterValue(handle, time, ADXL362_REG_TIME_ACT)) {
        return false;
    }

    // Enable activity interrupt and select a referenced or absolute configuration
    if(!ADXL362_getRegisterValue(handle, &l_oldActInactReg, ADXL362_REG_ACT_INACT_CTL)) {
        return false;
    }

    l_newActInactReg = l_oldActInactReg & ~ADXL362_ACT_INACT_CTL_ACT_REF;
    l_newActInactReg |= ADXL362_ACT_INACT_CTL_ACT_EN | (referenceMode * ADXL362_ACT_INACT_CTL_ACT_REF);

    return ADXL362_setRegisterValue(handle, l_newActInactReg, ADXL362_REG_ACT_INACT_CTL);

}

/**
 * @brief      Configures inactivity detection.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  bool referenceMode - Referenced/Absolute Inactivity Select. 'true' - referenced mode.
 *             'false' - absolute mode.
 * @param[in]  uint16_t threshold - 11-bit unsigned value that the adxl362 samples are compared to.
 * @param[in]  uint16_t time - 16-bit value written to the inactivity timer register. The amount of time (in seconds) is:
 *             time / ODR, where ODR - is the output data rate.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setupInactivityDetection(ADXL362_t *handle, bool referenceMode, uint16_t threshold, uint16_t time)
{
    if (!handle) {
        return false;
    }

    uint8_t l_oldActInactReg = 0;
    uint8_t l_newActInactReg = 0;
    uint16_t l_threshold = threshold & 0x7FF;

    // Configure motion threshold and inactivity timer
    if(!ADXL362_burstWite(handle, (uint8_t*)(&l_threshold), ADXL362_REG_THRESH_INACT_L, 2)) {
        return false;
    }
    if(!ADXL362_setRegisterValue(handle, time, ADXL362_REG_TIME_INACT_L)) {
        return false;
    }

    // Enable inactivity interrupt and select a referenced or absolute configuration
    if(!ADXL362_getRegisterValue(handle, &l_oldActInactReg, ADXL362_REG_ACT_INACT_CTL)) {
        return false;
    }

    l_newActInactReg = l_oldActInactReg & ~ADXL362_ACT_INACT_CTL_INACT_REF;
    l_newActInactReg |= ADXL362_ACT_INACT_CTL_INACT_EN | (referenceMode * ADXL362_ACT_INACT_CTL_INACT_REF);

    return ADXL362_setRegisterValue(handle, l_newActInactReg, ADXL362_REG_ACT_INACT_CTL);
}

/**
 * @brief      Configures Interrupt1 awake Mask.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  uint8_t awakeMap - AwakeMap bitmask. Combination of ADXL362_intMask_t types.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setIntMap1(ADXL362_t *handle, uint8_t awakeMap)
{
    if (!handle) {
        return false;
    }

    return ADXL362_setRegisterValue(handle, awakeMap, ADXL362_REG_INTMAP1);
}

/**
 * @brief      Configures Interrupt2 awake Mask.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  uint8_t awakeMap - AwakeMap bitmask. Combination of ADXL362_intMask_t types.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool ADXL362_setIntMap2(ADXL362_t *handle, uint8_t awakeMap)
{
    if (!handle) {
        return false;
    }

    return ADXL362_setRegisterValue(handle, awakeMap, ADXL362_REG_INTMAP2);
}


/* Private functions. */

/**
 * @brief      Write data to SPI.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362.
 * @param[in]  uint8_t adr - Address of the register.
 * @param[in]  const uint8_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be write.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADXL362_spiWrite(ADXL362_t *handle, uint8_t adr, const uint8_t *pData, uint16_t size)
{
    bool l_result = false;

    uint8_t l_header[2] = { ADXL362_WRITE_REG, adr };

    SeGpio_setOutPin(handle->m_cs, false);

    if (SeSpi_transmit(handle->m_hspi, l_header, 2) && SeSpi_transmit(handle->m_hspi, pData, size)) {
        l_result = true;
    }

    SeGpio_setOutPin(handle->m_cs, true);

    return l_result;
}

/**
 * @brief      Read data from SPI.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362
 * @param[in]  uint8_t adr - Address of the register.
 * @param[out] uint16_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be read.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADXL362_spiRead(ADXL362_t *handle, uint8_t adr, uint8_t *pData, uint16_t size)
{
    bool l_result = false;

    uint8_t l_header[2] = { ADXL362_READ_REG, adr };

    SeGpio_setOutPin(handle->m_cs, false);

    if (SeSpi_transmit(handle->m_hspi, l_header, 2) && SeSpi_receive(handle->m_hspi, pData, size)) {
        l_result = true;
    }

    SeGpio_setOutPin(handle->m_cs, true);

    return l_result;
}

/**
 * @brief      Read data from ADXL362 FIFO.
 * @param[in]  ADXL362_t *handle - Pointer to instance of ADXL362
 * @param[out] uint16_t *pData - Pointer to data.
 * @param[in]  uint16_t size - Amount of data to be read.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
static bool ADXL362_spiReadFifo(ADXL362_t *handle, uint8_t *pData, uint16_t size)
{
    bool l_result = false;

    uint8_t l_header = ADXL362_WRITE_FIFO;

    SeGpio_setOutPin(handle->m_cs, false);

    if (SeSpi_transmit(handle->m_hspi, &l_header, 1) && SeSpi_receive(handle->m_hspi, pData, size)) {
        l_result = true;
    }

    SeGpio_setOutPin(handle->m_cs, true);

    return l_result;
}

