/***************************************************************************//**
 * @file   SeSpi.h
 * @brief  SeSpi header file.
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

#ifndef _SE_SPI_H_
#define _SE_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif
     

#include "spidrv.h"

#include "Porting.h"


/* SPI mode. */
typedef struct {
    uint32_t            m_baudrate;       /* SPI speed. */
    uint32_t            m_frameLength;    /* An SPI framelength, valid numbers are 4..16. */
    SPIDRV_ClockMode_t  m_clockMode;      /* SPI mode, CLKPOL/CLKPHASE setting. */
    SPIDRV_BitOrder_t   m_bitOrder;       /* SPI bus bit order. */
} SeSpi_mode_t;

/* SPI driver instance initialization structure.
   This data structure contains a number of SPI configuration options
   required for driver instance initialization.
   This struct is passed to @ref SeSpi_init() when initializing a SeSpi
   instance. */
typedef struct {
    USART_TypeDef      *m_port;           /* Peripheral port. */
    SeSpi_mode_t        m_mode;           /* SPI mode. */
    uint8_t             m_clkLoc;         /* CLK pin location. */
    uint8_t             m_misoLoc;        /* MISO pin location. */
    uint8_t             m_mosiLoc;        /* MOSI pin location. */
} SeSpi_init_t;

/* Structure that contains the configuration information for SPI module. */
typedef struct {
    SPIDRV_HandleData_t m_instance;
} SeSpi_t;


/* Initialize the SPI according to the specified parameters. */
bool SeSpi_init(SeSpi_t *hspi, const SeSpi_init_t *init);

/* Transmit an amount of data in blocking mode. */
bool SeSpi_transmit(SeSpi_t *hspi, const uint8_t *pData, uint16_t size);

/* Receive an amount of data in blocking mode. */
bool SeSpi_receive(SeSpi_t *hspi, uint8_t *pData, uint16_t size);

/* Transmit and Receive an amount of data in blocking mode. */
bool SeSpi_transfer(SeSpi_t *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t size);

/* Set mode for transfer data. */
bool SeSpi_setMode(SeSpi_t *hspi, const SeSpi_mode_t *mode);

/* Get current mode. */
bool SeSpi_getMode(SeSpi_t *hspi, SeSpi_mode_t *mode);

 
#ifdef __cplusplus
}
#endif

#endif /* _SE_SPI_H_ */
