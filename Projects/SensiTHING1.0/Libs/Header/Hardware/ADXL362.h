/***************************************************************************//**
 * @file   ADXL362.h
 * @brief  Header file of ADXL362 Driver.
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

#ifndef _ADXL362_H_
#define _ADXL362_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Porting.h"
#include "SeSpi.h"
#include "SeGpio.h"


/* ADXL362 registers. */
typedef enum {
    ADXL362_REG_DEVID_AD            = 0x00,
    ADXL362_REG_DEVID_MST           = 0x01,
    ADXL362_REG_PARTID              = 0x02,
    ADXL362_REG_REVID               = 0x03,
    ADXL362_REG_XDATA               = 0x08,
    ADXL362_REG_YDATA               = 0x09,
    ADXL362_REG_ZDATA               = 0x0A,
    ADXL362_REG_STATUS              = 0x0B,
    ADXL362_REG_FIFO_L              = 0x0C,
    ADXL362_REG_FIFO_H              = 0x0D,
    ADXL362_REG_XDATA_L             = 0x0E,
    ADXL362_REG_XDATA_H             = 0x0F,
    ADXL362_REG_YDATA_L             = 0x10,
    ADXL362_REG_YDATA_H             = 0x11,
    ADXL362_REG_ZDATA_L             = 0x12,
    ADXL362_REG_ZDATA_H             = 0x13,
    ADXL362_REG_TEMP_L              = 0x14,
    ADXL362_REG_TEMP_H              = 0x15,
    ADXL362_REG_SOFT_RESET          = 0x1F,
    ADXL362_REG_THRESH_ACT_L        = 0x20,
    ADXL362_REG_THRESH_ACT_H        = 0x21,
    ADXL362_REG_TIME_ACT            = 0x22,
    ADXL362_REG_THRESH_INACT_L      = 0x23,
    ADXL362_REG_THRESH_INACT_H      = 0x24,
    ADXL362_REG_TIME_INACT_L        = 0x25,
    ADXL362_REG_TIME_INACT_H        = 0x26,
    ADXL362_REG_ACT_INACT_CTL       = 0x27,
    ADXL362_REG_FIFO_CTL            = 0x28,
    ADXL362_REG_FIFO_SAMPLES        = 0x29,
    ADXL362_REG_INTMAP1             = 0x2A,
    ADXL362_REG_INTMAP2             = 0x2B,
    ADXL362_REG_FILTER_CTL          = 0x2C,
    ADXL362_REG_POWER_CTL           = 0x2D,
    ADXL362_REG_SELF_TEST           = 0x2E,
} ADXL362_registers_t;

/* ADXL362 power mode. */
typedef enum {
    ADXL362_PWR_STANDBY             = 0,
    ADXL362_PWR_MEASURE             = 1,
} ADXL362_powerMode_t;

/* ADXL362 sensitivity range options. */
typedef enum {
    ADXL362_RANGE_2G                = 0,    /* +/-2 g */
    ADXL362_RANGE_4G                = 1,    /* +/-4 g */
    ADXL362_RANGE_8G                = 2,    /* +/-8 g */
} ADXL362_range_t;

/* ADXL362 output data rate options. */
typedef enum {
    ADXL362_ODR_12_5_HZ             = 0,    /* 12.5 Hz */
    ADXL362_ODR_25_HZ               = 1,    /* 25 Hz */
    ADXL362_ODR_50_HZ               = 2,    /* 50 Hz */
    ADXL362_ODR_100_HZ              = 3,    /* 100 Hz */
    ADXL362_ODR_200_HZ              = 4,    /* 200 Hz */
    ADXL362_ODR_400_HZ              = 5,    /* 400 Hz */
} ADXL362_outDataRate_t;

/* ADXL362 output data rate options. */
typedef enum {
    ADXL362_FIFO_DISABLE            = 0,
    ADXL362_FIFO_OLDEST_SAVED       = 1,
    ADXL362_FIFO_STREAM             = 2,
    ADXL362_FIFO_TRIGGERED          = 3,
} ADXL362_fifoMode_t;

/* ADXL362 Interrupt mask. */
typedef enum {
    ADXL362_INTMAP_INT_LOW          = (1 << 7),
    ADXL362_INTMAP_AWAKE            = (1 << 6),
    ADXL362_INTMAP_INACT            = (1 << 5),
    ADXL362_INTMAP_ACT              = (1 << 4),
    ADXL362_INTMAP_FIFO_OVERRUN     = (1 << 3),
    ADXL362_INTMAP_FIFO_WATERMARK   = (1 << 2),
    ADXL362_INTMAP_FIFO_READY       = (1 << 1),
    ADXL362_INTMAP_DATA_READY       = (1 << 0),
} ADXL362_intMask_t;

/* ADXL362_REG_STATUS register structure. */
typedef union {
    struct {
        uint8_t DataReady:      1;
        uint8_t FifoReady:      1;
        uint8_t FifoWatermark:  1;
        uint8_t FifoOverrun:    1;
        uint8_t Active:         1;
        uint8_t Inactive:       1;
        uint8_t Awake:          1;
        uint8_t ErrUserRegs:    1;
    } bits;
    uint8_t reg;
} ADXL362_statusReg_t;


/* Structure that contains the configuration information for ADXL362. */
typedef struct {
    const SeGpio_pin_t  *m_cs;
    SeSpi_t             *m_hspi;
    uint8_t             m_address;
    uint8_t             m_range;
}  ADXL362_t;


/* Initialize ADXL362. */
bool ADXL362_init(ADXL362_t *handle, const SeGpio_pin_t *cs, SeSpi_t *hspi);

/* Checking Component presence reading the device IDs. */
bool ADXL362_check(ADXL362_t *handle);

/* Writes data into a register. */
bool ADXL362_setRegisterValue(ADXL362_t *handle, uint8_t regValue, ADXL362_registers_t reg);

/* Performs a burst read of a specified number of registers. */
bool ADXL362_getRegisterValue(ADXL362_t *handle, uint8_t *pReadData, ADXL362_registers_t reg);

/* Get status. */
bool ADXL362_getStatus(ADXL362_t *handle, ADXL362_statusReg_t *status);

/* Performs a burst write of a specified number of registers. */
bool ADXL362_burstWite(ADXL362_t *handle, const uint8_t *pWriteData, ADXL362_registers_t reg, uint16_t bytesNumber);

/* Performs a burst read of a specified number of registers. */
bool ADXL362_burstRead(ADXL362_t *handle, uint8_t *pReadData, ADXL362_registers_t reg, uint16_t bytesNumber);

/* Reads multiple bytes from the device's FIFO buffer. */
bool ADXL362_getFifoValue(ADXL362_t *handle, uint8_t *pBuffer, uint16_t bytesNumber);

/* Resets the device via SPI communication bus. */
bool ADXL362_softwareReset(ADXL362_t *handle);

/* Places the device into standby or measure mode. */
bool ADXL362_setPowerMode(ADXL362_t *handle, ADXL362_powerMode_t pwrMode);

/* Places the device into wakeup mode. */
bool ADXL362_setWakeupMode(ADXL362_t *handle, bool wakeup);

/* Selects the measurement range. */
bool ADXL362_setRange(ADXL362_t *handle, ADXL362_range_t gRange);

/* Selects the Output Data Rate of the device. */
bool ADXL362_setOutputRate(ADXL362_t *handle, ADXL362_outDataRate_t outRate);

/* Reads the 3-axis raw data from the accelerometer. */
bool ADXL362_getXyz(ADXL362_t *handle, int16_t *x, int16_t *y, int16_t *z);

/* Reads the 3-axis raw data from the accelerometer and converts it to g. */
bool ADXL362_getGxyz(ADXL362_t *handle, float *x, float *y, float *z);

/* Reads the temperature of the device. */
bool ADXL362_readTemperature(ADXL362_t *handle, float *temperature);

/* Configures the FIFO feature. */
bool ADXL362_fifoSetup(ADXL362_t *handle, ADXL362_fifoMode_t mode, uint16_t waterMarkLvl, bool includeTemp);

/* Configures activity detection. */
bool ADXL362_setupActivityDetection(ADXL362_t *handle, bool referenceMode, uint16_t threshold, uint8_t  time);

/* Configures inactivity detection. */
bool ADXL362_setupInactivityDetection(ADXL362_t *handle, bool referenceMode, uint16_t threshold, uint16_t time);

/* Configures Interrupt1 awake Mask. */
bool ADXL362_setIntMap1(ADXL362_t *handle, uint8_t  awakeMap);

/* Configures Interrupt2 awake Mask. */
bool ADXL362_setIntMap2(ADXL362_t *handle, uint8_t  awakeMap);



#ifdef __cplusplus
}
#endif

#endif /* _ADXL362_H_ */
