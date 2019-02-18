/***************************************************************************//**
 * @file   AD5592R.h
 * @brief  Header file of AD5592R Driver.
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

#ifndef _AD5592R_H_
#define _AD5592R_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Porting.h"
#include "SeSpi.h"
#include "SeGpio.h"


/* AD5592R registers. */
typedef enum {
    AD5592R_REG_NOP             = 0x0,
    AD5592R_REG_DAC_READBACK    = 0x1,
    AD5592R_REG_ADC_SEQ         = 0x2,
    AD5592R_REG_CTRL            = 0x3,
    AD5592R_REG_ADC_EN          = 0x4,
    AD5592R_REG_DAC_EN          = 0x5,
    AD5592R_REG_PULLDOWN        = 0x6,
    AD5592R_REG_LDAC            = 0x7,
    AD5592R_REG_GPIO_OUT_EN     = 0x8,
    AD5592R_REG_GPIO_SET        = 0x9,
    AD5592R_REG_GPIO_IN_EN      = 0xA,
    AD5592R_REG_PD              = 0xB,
    AD5592R_REG_OPEN_DRAIN      = 0xC,
    AD5592R_REG_TRISTATE        = 0xD,
    AD5592R_REG_RESET           = 0xF,
} AD5592R_registers_t;

/* Determines how data written to an input register of a DAC is handled. */
typedef enum {
    AD5592R_DAC_MODE_IMMEDIATELY_UPD = 0,
    AD5592R_DAC_MODE_NOT_UPD,
} AD5592R_ldacMode_t;

/* ADC/DAC gain. */
typedef enum {
    AD5592R_GAIN_1 = 1,
    AD5592R_GAIN_2 = 2,
} AD5592R_gain_t;

/* Pin mode for AD5592R I/O. */
typedef enum {
    AD5592R_PIN_MODE_PULLDOWN = 0,
    AD5592R_PIN_MODE_DAC,
    AD5592R_PIN_MODE_ADC,
    AD5592R_PIN_MODE_DAC_ADC,
    AD5592R_PIN_MODE_GPIO_IN,
    AD5592R_PIN_MODE_GPIO_OUT,
    AD5592R_PIN_MODE_GPIO_IN_OUT,
    AD5592R_PIN_MODE_OPEN_DRAIN,
    AD5592R_PIN_MODE_TRISTATE,
} AD5592R_pinMode_t;

/* Structure that contains the configuration information for AD5592R. */
typedef struct {
    const SeGpio_pin_t  *m_cs;
    SeSpi_t             *m_hspi;

    // Saved values to minimize read-modify-write operations
    AD5592R_ldacMode_t  m_ldacMode;
    AD5592R_gain_t      m_adcGain;

} AD5592R_t;


/* Initialize AD5592R. */
bool AD5592R_init(AD5592R_t *handle, const SeGpio_pin_t *cs, SeSpi_t *hspi);

/* Common configuration. */
bool AD5592R_softReset(AD5592R_t *handle);
bool AD5592R_powerDown(AD5592R_t *handle, bool value);
bool AD5592R_setIntReference(AD5592R_t *handle, bool value);
bool AD5592R_confPin(AD5592R_t *handle, uint8_t pin, AD5592R_pinMode_t mode);

/* GPIO functions. */
bool AD5592R_gpioWriteData(AD5592R_t *handle, uint8_t pin, bool value);
bool AD5592R_gpioReadData(AD5592R_t *handle, uint8_t pin, bool *value );

/* ADC function. */
bool AD5592R_adcEnableBuffer(AD5592R_t *handle, bool value);
bool AD5592R_adcSetGain(AD5592R_t *handle, AD5592R_gain_t gain);
bool AD5592R_adcReadData(AD5592R_t *handle, uint8_t pin, uint16_t *value);
bool AD5592R_adcReadTemperature(AD5592R_t *handle, float *value);

/* DAC function. */
bool AD5592R_dacSetMode(AD5592R_t *handle, AD5592R_ldacMode_t mode);
bool AD5592R_dacUpdateOutputs(AD5592R_t *handle);
bool AD5592R_dacSetGain(AD5592R_t *handle, AD5592R_gain_t gain);
bool AD5592R_dacWriteData(AD5592R_t *handle, uint8_t pin, uint16_t value);
bool AD5592R_dacReadData(AD5592R_t *handle, uint8_t pin, uint16_t *value);

/* Work with raw data. */
bool AD5592R_setRegisterValue(AD5592R_t *handle, uint16_t value, AD5592R_registers_t reg);
bool AD5592R_getRegisterValue(AD5592R_t *handle, uint16_t *value, AD5592R_registers_t reg);


#ifdef __cplusplus
}
#endif

#endif /* _AD5592R_H_ */
