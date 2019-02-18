/***************************************************************************//**
 * @file   SeSpi.c
 * @brief  Functions and data related to SeSpi.
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

#include "SeSpi.h"


/* Public functions. */

/**
 * @brief      Initialize the SPI according to the specified parameters.
 * @param[in]  SeSpi_t *hspi - Pointer to a SeSpi_t structure that contains
 *             the configuration information for SPI module.
 * @param[in]  const SeSpi_init_t *init - Pointer to SPI initialization structure.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeSpi_init(SeSpi_t *hspi, const SeSpi_init_t *init)
{
    if(!hspi || !init) {
        return false;
    }

    // Any usart as default config
    SPIDRV_Init_t l_initData = SPIDRV_MASTER_USART0;

    l_initData.port = init->m_port;
    l_initData.portLocationClk = init->m_clkLoc;
    l_initData.portLocationRx = init->m_misoLoc;
    l_initData.portLocationTx = init->m_mosiLoc;
    l_initData.csControl = spidrvCsControlApplication;

    l_initData.bitRate = init->m_mode.m_baudrate;
    l_initData.frameLength = init->m_mode.m_frameLength;
    l_initData.clockMode = init->m_mode.m_clockMode;
    l_initData.bitOrder = init->m_mode.m_bitOrder;

    // Initialize a SPI driver instance
    return (SPIDRV_Init(&hspi->m_instance, &l_initData ) == ECODE_OK);
}

/**
 * @brief      Transmit an amount of data in blocking mode.
 * @param[in]  SeSpi_t *hspi - Pointer to a SeSpi_t structure that contains
 *             the configuration information for SPI module.
 * @param[in]  const uint8_t *pData - Pointer to data buffer.
 * @param[in]  uint16_t size -  Amount of data to be sent.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeSpi_transmit(SeSpi_t *hspi, const uint8_t *pData, uint16_t size)
{
    if(!hspi || !pData) {
        return false;
    }

    bool l_result = (SPIDRV_MTransmitB(&hspi->m_instance, pData, size) == ECODE_OK);

    if(l_result) {
        // Wait till data is written
        while (!(hspi->m_instance.initData.port->STATUS & USART_STATUS_TXC));
    }

    return l_result;
}

/**
 * @brief      Receive an amount of data in blocking mode.
 * @param[in]  SeSpi_t *hspi - Pointer to a SeSpi_t structure that contains
 *             the configuration information for SPI module.
 * @param[out] uint8_t *pData - Pointer to data buffer.
 * @param[in]  uint16_t size Amount of data to be received.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeSpi_receive(SeSpi_t *hspi, uint8_t *pData, uint16_t size)
{
    if(!hspi || !pData) {
        return false;
    }

    bool l_result = (SPIDRV_MReceiveB(&hspi->m_instance, pData, size) == ECODE_OK);

    if(l_result) {
        // Wait till data is reading
        while (!(hspi->m_instance.initData.port->STATUS & USART_STATUS_TXC));
    }

    return l_result;
}

/**
 * @brief      Transmit and Receive an amount of data in blocking mode.
 * @param[in]  SeSpi_t *hspi - Pointer to a SeSpi_t structure that contains
 *             the configuration information for SPI module.
 * @param[in]  const uint8_t *pTxData - Pointer to transmission data buffer.
 * @param[out] uint8_t *pRxData - Pointer to reception data buffer.
 * @param[in]  uint16_t size - Amount of data to be sent and received.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeSpi_transfer(SeSpi_t *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
    if(!hspi || !pTxData || !pRxData) {
        return false;
    }

    return (SPIDRV_MTransferB(&hspi->m_instance, pTxData, pRxData, size) == ECODE_OK);
}

/**
 * @brief      Set mode for transfer data.
 * @param[in]  SeSpi_t *hspi - Pointer to a SeSpi_t structure that contains
 *             the configuration information for SPI module.
 * @param[in]  const SeSpi_mode_t *mode - Pointer to new SPI mode.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeSpi_setMode(SeSpi_t *hspi, const SeSpi_mode_t *mode)
{
    if(!hspi || !mode) {
        return false;
    }

    SPIDRV_Init_t l_initData;

    // Restore init data
    memcpy(&l_initData, &hspi->m_instance.initData, sizeof(SPIDRV_Init_t));

    if ((l_initData.frameLength != mode->m_frameLength) ||
            (l_initData.bitRate != mode->m_baudrate) ||
            (l_initData.clockMode != mode->m_clockMode) ||
            (l_initData.bitOrder != mode->m_bitOrder))
    {

        l_initData.frameLength = mode->m_frameLength;
        l_initData.bitRate = mode->m_baudrate;
        l_initData.clockMode = mode->m_clockMode;
        l_initData.bitOrder = mode->m_bitOrder;

        SPIDRV_DeInit(&hspi->m_instance);
        return (SPIDRV_Init(&hspi->m_instance, &l_initData) == ECODE_OK);
    }

    return true;
}

/**
 * @brief      Get current mode.
 * @param[in]  SeSpi_t *hspi - Pointer to a SeSpi_t structure that contains
 *             the configuration information for SPI module.
 * @param[in]  SeSpi_mode_t *mode - Ppointer to new SPI mode.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeSpi_getMode(SeSpi_t *hspi, SeSpi_mode_t *mode)
{
    if(!hspi || !mode) {
        return false;
    }

    SPIDRV_Init_t l_initData;

    // Restore init data
    memcpy(&l_initData, &hspi->m_instance.initData, sizeof(SPIDRV_Init_t));

    mode->m_frameLength = l_initData.frameLength;
    mode->m_baudrate = l_initData.bitRate;
    mode->m_clockMode = l_initData.clockMode;
    mode->m_bitOrder = l_initData.bitOrder;

    return true;
}
