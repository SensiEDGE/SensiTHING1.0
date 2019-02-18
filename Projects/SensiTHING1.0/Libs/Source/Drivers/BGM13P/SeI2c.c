/***************************************************************************//**
 * @file   SeI2c.c
 * @brief  Functions and data related to SeI2c.
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

#include "em_cmu.h"
#include "em_gpio.h"

#include "SeI2c.h"


/* Private defines. */

/* Timeout for data transfering. */
#define I2C_TRANSFER_TIMEOUT 300000


/* Public functions. */

/**
 * @brief      Initialize the I2C according to the specified parameters.
 * @param[in]  SeI2c_t *hi2c - Pointer to a SeI2c_t structure that contains
 *             the configuration information for I2C module.
 * @param[in]  const SeI2c_init_t *init - Pointer to I2C initialization structure.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeI2c_init(SeI2c_t *hi2c, const SeI2c_init_t *init)
{
    if(!hi2c || !init) {
        return false;
    }

    CMU_Clock_TypeDef i2cClock;
    I2C_Init_TypeDef i2cInit;
    GPIO_Port_TypeDef l_sclPort;
    uint16_t l_sclPin;
    GPIO_Port_TypeDef l_sdaPort;
    uint16_t l_sdaPin;

    CMU_ClockEnable(cmuClock_HFPER, true);

    // Select I2C peripheral clock
    if(false){

    }
    #if defined(I2C0)
    else if (init->m_port == I2C0) {
        i2cClock = cmuClock_I2C0;
        l_sclPort = AF_I2C0_SCL_PORT(init->m_sclLoc);
        l_sclPin = AF_I2C0_SCL_PIN(init->m_sclLoc);

        l_sdaPort = AF_I2C0_SDA_PORT(init->m_sdaLoc);
        l_sdaPin = AF_I2C0_SDA_PIN(init->m_sdaLoc);
    }
    #endif
    #if defined(I2C1)
    else if (init->m_port == I2C1) {
        i2cClock = cmuClock_I2C1;
        l_sclPort = AF_I2C1_SCL_PORT(init->m_sclLoc);
        l_sclPin = AF_I2C1_SCL_PIN(init->m_sclLoc);

        l_sdaPort = AF_I2C1_SDA_PORT(init->m_sdaLoc);
        l_sdaPin = AF_I2C1_SDA_PIN(init->m_sdaLoc);
    }
    #endif
    #if defined(I2C2)
    else if (init->m_port == I2C2) {
        i2cClock = cmuClock_I2C2;
        l_sclPort = AF_I2C2_SCL_PORT(init->m_sclLoc);
        l_sclPin = AF_I2C2_SCL_PIN(init->m_sclLoc);

        l_sdaPort = AF_I2C2_SDA_PORT(init->m_sdaLoc);
        l_sdaPin = AF_I2C2_SDA_PIN(init->m_sdaLoc);
    }
    #endif
    else {
        // I2C clock is not defined
        return false;
    }

    hi2c->m_port = init->m_port;

    CMU_ClockEnable(i2cClock, true);

    GPIO_PinModeSet(l_sclPort, l_sclPin, gpioModeWiredAndPullUp, 1);
    GPIO_PinModeSet(l_sdaPort, l_sdaPin, gpioModeWiredAndPullUp, 1);

    /* In some situations, after a reset during an I2C transfer, the slave
       device may be left in an unknown state. Send 9 clock pulses to
       set slave in a defined state. */
    for (int i = 0; i < 9; i++) {
      GPIO_PinOutSet(l_sclPort, l_sclPin);
      GPIO_PinOutClear(l_sclPort, l_sclPin);
    }

    // Enable pins and set location
  #if defined (_I2C_ROUTEPEN_MASK)
    init->m_port->ROUTEPEN  = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
    init->m_port->ROUTELOC0 = (init->m_sdaLoc << _I2C_ROUTELOC0_SDALOC_SHIFT)
                            | (init->m_sclLoc << _I2C_ROUTELOC0_SCLLOC_SHIFT);
  #else
    init->port->ROUTE = I2C_ROUTE_SDAPEN
                        | I2C_ROUTE_SCLPEN
                        | (init->portLocation << _I2C_ROUTE_LOCATION_SHIFT);
  #endif

    // Set emlib init parameters
    i2cInit.enable = true;
    i2cInit.master = true;
    i2cInit.freq = init->m_i2cMaxFreq;
    i2cInit.refFreq = init->m_i2cRefFreq;
    i2cInit.clhr = init->m_i2cClhr;

    I2C_Init(init->m_port, &i2cInit);

    return true;
}

/**
 * @brief      Transmit in master mode an amount of data in blocking mode.
 * @param[in]  SeI2c_t *hi2c - Pointer to a SeI2c_t structure that contains
 *             the configuration information for I2C module.
 * @param[in]  uint8_t devAddress - Target device address. 7-bit.
 * @param[in]  const uint8_t *pDataTx - Pointer to transmit data buffer.
 * @param[in]  uint16_t sizeTx - Amount of data to be sent (in bytes).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeI2c_transmit(SeI2c_t* hi2c, uint16_t devAddress, const uint8_t* pDataTx, uint16_t sizeTx)
{
    return SeI2c_transfer(hi2c, devAddress, pDataTx, sizeTx, 0, 0);
}

/**
 * @brief      Receive in master mode an amount of data in blocking mode.
 * @param[in]  SeI2c_t *hi2c - Pointer to a SeI2c_t structure that contains
 *             the configuration information for I2C module.
 * @param[in]  uint8_t devAddress - Target device address. 7-bit.
 * @param[out] uint8_t *pDataRx - Pointer to receive data buffer.
 * @param[in]  uint16_t sizeRx - amount of data to be received (in bytes).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeI2c_receive(SeI2c_t* hi2c, uint16_t devAddress, uint8_t* pDataRx, uint16_t sizeRx)
{
    return SeI2c_transfer(hi2c,devAddress, 0, 0, pDataRx, sizeRx);
}

/**
 * @brief      Transmit and Receive in master mode an amount of data in blocking mode.
 * @param[in]  SeI2c_t *hi2c - Pointer to a SeI2c_t structure that contains
 *             the configuration information for I2C module.
 * @param[in]  uint8_t devAddress - Target device address. 7-bit.
 * @param[in]  const uint8_t *pDataTx - Pointer to transmit data buffer.
 * @param[in]  uint16_t sizeTx - Amount of data to be sent (in bytes).
 * @param[out] uint8_t *pDataRx - Pointer to receive data buffer.
 * @param[in]  uint16_t sizeRx - amount of data to be received (in bytes).
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeI2c_transfer(SeI2c_t *hi2c, uint8_t devAddress, const uint8_t *pDataTx, uint16_t sizeTx, uint8_t *pDataRx, uint16_t sizeRx)
{
    if(!hi2c || (!pDataTx && !pDataRx)) {
        return false;
    }

    I2C_TransferReturn_TypeDef l_ret = 0;
    I2C_TransferSeq_TypeDef l_seq;
    uint8_t l_dummy[2];
    uint32_t timeout = I2C_TRANSFER_TIMEOUT;

    l_seq.addr = (devAddress << 1);

    // Select location/length of data to be write
     if(pDataTx) {
         l_seq.buf[0].data   = (uint8_t*)pDataTx;
         l_seq.buf[0].len    = sizeTx;
         l_seq.flags = I2C_FLAG_WRITE;
     } else {
         l_seq.buf[0].data   = l_dummy;
         l_seq.buf[0].len    = 0;
     }
     // Select location/length of data to be read
     if(pDataRx) {
         l_seq.buf[1].data = pDataRx;
         l_seq.buf[1].len  = sizeRx;
         l_seq.flags = I2C_FLAG_READ;
     } else {
         l_seq.buf[1].data = l_dummy;
         l_seq.buf[1].len  = 1;
     }
    if(pDataTx && pDataRx) {
        // Overwrite flag
        l_seq.flags = I2C_FLAG_WRITE_READ;
    }


    // Do a polled transfer
    l_ret = I2C_TransferInit(hi2c->m_port, &l_seq);
    while (l_ret == i2cTransferInProgress && timeout--) {
        l_ret = I2C_Transfer(hi2c->m_port);
    }

    return (l_ret == i2cTransferDone);
}
