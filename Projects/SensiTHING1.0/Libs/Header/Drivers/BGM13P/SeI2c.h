/***************************************************************************//**
 * @file   SeI2c.h
 * @brief  SeI2c header file
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

#ifndef _SE_I2C_H_
#define _SE_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif
     

#include "em_i2c.h"

#include "Porting.h"


/* I2C driver instance initialization structure.
   This data structure contains a number of I2C configuration options
   required for driver instance initialization.
   This struct is passed to @ref SeI2c_init() when initializing a SeI2c
   instance. */
typedef struct {
  I2C_TypeDef           *m_port;          /* Peripheral port. */
  uint8_t               m_sclLoc;         /* SCL pin location. */
  uint8_t               m_sdaLoc;         /* SDA pin location. */
  uint32_t              m_i2cRefFreq;     /* I2C reference clock. */
  uint32_t              m_i2cMaxFreq;     /* I2C max bus frequency to use. */
  I2C_ClockHLR_TypeDef  m_i2cClhr;        /* Clock low/high ratio control. */
} SeI2c_init_t;

/* Structure that contains the configuration information for I2C module. */
typedef struct {
    I2C_TypeDef         *m_port;          /* Peripheral port. */
} SeI2c_t;


/* Initialize the I2C according to the specified parameters. */
bool SeI2c_init(SeI2c_t *hi2c, const SeI2c_init_t *init);

/* Transmit in master mode an amount of data in blocking mode. */
bool SeI2c_transmit(SeI2c_t* hi2c, uint16_t devAddress, const uint8_t* pDataTx, uint16_t sizeTx);

/* Receive in master mode an amount of data in blocking mode. */
bool SeI2c_receive(SeI2c_t* hi2c, uint16_t devAddress, uint8_t* pDataRx, uint16_t sizeRx);

/* Transmit and Receive in master mode an amount of data in blocking mode. */
bool SeI2c_transfer(SeI2c_t *hi2c, uint8_t devAddress, const uint8_t *pDataTx, uint16_t sizeTx, uint8_t *pDataRx, uint16_t sizeRx);


#ifdef __cplusplus
}
#endif

#endif /* _SE_I2C_H_ */
