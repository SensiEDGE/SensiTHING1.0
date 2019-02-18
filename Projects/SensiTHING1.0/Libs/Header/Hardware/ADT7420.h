/***************************************************************************//**
 * @file   ADT7420.h
 * @brief  Header file of ADT7420 Driver.
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

#ifndef _ADT7420_H_
#define _ADT7420_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Porting.h"
#include "SeI2c.h"


/* ADT7420 registers. */
typedef enum {
    ADT7420_REG_TEMP_MSB                = 0x00,
    ADT7420_REG_TEMP_LSB                = 0x01,
    ADT7420_REG_STATUS                  = 0x02,
    ADT7420_REG_CONFIG                  = 0x03,
    ADT7420_REG_T_HIGH_SETPOINT_MSB     = 0x04,
    ADT7420_REG_T_HIGH_SETPOINT_LSB     = 0x05,
    ADT7420_REG_T_LOW_SETPOINT_MSB      = 0x06,
    ADT7420_REG_T_LOW_SETPOINT_LSB      = 0x07,
    ADT7420_REG_T_CRIT_SETPOINT_MSB     = 0x08,
    ADT7420_REG_T_CRIT_SETPOINT_LSB     = 0x09,
    ADT7420_REG_T_HYST_SETPOINT         = 0x0A,
    ADT7420_REG_ID                      = 0x0B,
    ADT7420_REG_SOFTWARE_RESET          = 0x2F,
} ADT7420_registers_t;

/* Temperature sensor resolution. */
typedef enum {
    ADT7420_RESOLUTION_13BIT    = 0,
    ADT7420_RESOLUTION_16BIT    = 1,
} ADT7420_resolution_t;

/* Temperature sensor operation mode. */
typedef enum {
    ADT7420_OP_CONTINUOUS   = 0,    /* Continuous conversion (default). When one conversion
                                       is finished, the ADT7420 starts another. */
    ADT7420_OP_ONE_SHOT     = 1,    /* One shot. Conversion time is typically 240 ms. */
    ADT7420_OP_SPS          = 2,    /* SPS mode. Conversion time is typically 60 ms. This operational
                                       mode reduces the average current consumption. */
    ADT7420_OP_SHUTDOWN     = 3,    /* Shutdown. All circuitry except interface circuitry is powered
                                       down. */
} ADT7420_operationMode_t;

/* INT pin mode. */
typedef enum {
    ADT7420_INT_INTERRUPT   = 0,    /* Interrupt mode. */
    ADT7420_INT_COMPARATOR  = 1,    /* Comparator mode. */
} ADT7420_intMode_t;


/* Pin polarity. */
typedef enum {
    ADT7420_POL_LOW     = 0,
    ADT7420_POL_HIGH    = 1,
} ADT7420_polarity_t;

/* Fault queue size. */
typedef enum {
    ADT7420_FAULT_QUEUE_1   = 0,
    ADT7420_FAULT_QUEUE_2   = 1,
    ADT7420_FAULT_QUEUE_3   = 2,
    ADT7420_FAULT_QUEUE_4   = 3,
} ADT7420_faultQueue_t;

/* Temperature setpoints. */
typedef enum {
    ADT7420_SETPOINT_T_HIGH = 0,
    ADT7420_SETPOINT_T_LOW  = 1,
    ADT7420_SETPOINT_T_CRIT = 2,
} ADT7420_setpoint_t;

/* ADT7420_REG_CONFIG register structure. */
typedef union {
    struct {
        ADT7420_faultQueue_t    FaultQueue:     2;
        ADT7420_polarity_t      CtPol:          1;
        ADT7420_polarity_t      IntPol:         1;
        ADT7420_intMode_t       IntCtMode:      1;
        ADT7420_operationMode_t OpMode:         2;
        ADT7420_resolution_t    Resolution:     1;
    } bits;
    uint8_t reg;
} ADT7420_configReg_t;

/* ADT7420_REG_STATUS register structure. */
typedef union {
    struct {
        uint8_t :       4;
        uint8_t TLow:   1;
        uint8_t THigh:  1;
        uint8_t TCrit:  1;
        uint8_t RDY:    1;  /* This bit goes low when the temperature conversion result is
                               written into the temperature value register. It is reset to '1' when
                               the temperature value register is read. In one-shot and '1' SPS modes,
                               this bit is reset after a write to the operation mode bits. */
    } bits;
    uint8_t reg;
} ADT7420_statusReg_t;

/* Structure that contains the configuration information for ADT7420. */
typedef struct {
    SeI2c_t                 *m_hi2c;
    uint8_t                 m_address;      /* I2C address. */
    ADT7420_resolution_t    m_resolution;   /* This value is necessary for converting the raw temperature
                                               in a hex into a float. The value is saved automatically
                                               when writing the configuration register. */
} ADT7420_t;


/* Initialize ADT7420. */
bool ADT7420_init(ADT7420_t *handle, SeI2c_t *hi2c, uint8_t address);

/* Work with raw data */
bool ADT7420_setRegisterValue(ADT7420_t *handle, ADT7420_registers_t reg, uint8_t value);
bool ADT7420_getRegisterValue(ADT7420_t *handle, ADT7420_registers_t reg, uint8_t *value);

/* Convert the raw temperature in a hex into a float. */
float ADT7420_convertHexToDegrees (uint16_t rawHex, ADT7420_resolution_t resolution);

/* Convert the temperature in a float into a hex. */
uint16_t ADT7420_convertDegreesToHex (float temperature, ADT7420_resolution_t resolution);

/* Set configuration. */
bool ADT7420_setConfig(ADT7420_t *handle, ADT7420_configReg_t config);

/* Get configuration. */
bool ADT7420_getConfig(ADT7420_t *handle, ADT7420_configReg_t *config);

/* Get status. */
bool ADT7420_getStatus(ADT7420_t *handle, ADT7420_statusReg_t *status);

/* Check if new data is ready. */
bool ADT7420_dataReady(ADT7420_t *handle);

/* Set operation mode. */
bool ADT7420_setOperationMode(ADT7420_t *handle, ADT7420_operationMode_t mode);

/* Get temperature. */
bool ADT7420_getTemperature(ADT7420_t *handle, float *temperature);

/* Set point of temperature. */
bool ADT7420_setPoint(ADT7420_t *handle, ADT7420_setpoint_t setpoint, float temperature);

/* Set hysteresis. */
bool ADT7420_setHysteresis(ADT7420_t *handle, uint8_t hyst);



#ifdef __cplusplus
}
#endif

#endif /* _ADT7420_H_ */
