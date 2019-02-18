/***************************************************************************//**
 * @file   Standby.c
 * @brief  Functions and data related to standby mode.
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

#include "em_emu.h"
#include "em_rmu.h"
#include "sleep.h"

#include "hal-config.h"
#include "Standby.h"


/* Public functions */

/**
 * @brief      Check if system boot from standby mode.
 * @retval     bool - 'true' - if system boot from standby, 'false' - otherwise.
 */
bool Standby_isStartFromStandby(void)
{
#if defined(RMU_RSTCAUSE_EM4WURST)
    if ((RMU_ResetCauseGet() & RMU_RSTCAUSE_EM4WURST) != 0)
#elif defined(RMU_RSTCAUSE_EM4RST)
    if ((RMU_ResetCauseGet() & RMU_RSTCAUSE_EM4RST) != 0)
#endif
    { // EM4 wake up
    	EMU_UnlatchPinRetention();
        return true;
    } else { // Other reset cause
        return false;
    }
}

/**
 * @brief      Enter to standby mode.
 * @retval     None.
 */
void Standby_enter(void)
{
    // Config EM4 sleep mode
    EMU_EM4Init_TypeDef init_EM4 = EMU_EM4INIT_DEFAULT;
    //init_EM4.em4State = emuEM4Hibernate;
    init_EM4.em4State = emuEM4Shutoff;
    init_EM4.pinRetentionMode = emuPinRetentionLatch;
    EMU_EM4Init(&init_EM4);

    // Init pin to start from sleep mode
    GPIO_PinModeSet(BSP_BUTTON_PORT, BSP_BUTTON_PIN, gpioModeInputPullFilter, 1);
    GPIO_EM4EnablePinWakeup( BSP_WAKEUP_LINE, BSP_WAKEUP_POLARITY);

    GPIO_IntClear(_GPIO_IFC_EM4WU_MASK | _GPIO_IFC_EXT_MASK);
    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

    // Set other pins
    GPIO_PinModeSet(gpioPortD, 13, gpioModeInput, 0);			// CHRG
    GPIO_PinModeSet(gpioPortD, 14, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);         // SPI1 CS W25Q80
    GPIO_PinModeSet(gpioPortA, 3, gpioModeInput, 0);			// ADXL362 INT
    //GPIO_PinModeSet(gpioPortA, 4, gpioModePushPull, 1);		// I2C2 SCL
    //GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 1);		// AD RST
    //GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, 1);		// SPI1 CS AD5592

    //GPIO_PinModeSet(gpioPortB, 13, gpioModePushPull, 1);		// I2C2 SDA
    //GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 1);		// SPI1 MOSI
    //GPIO_PinModeSet(gpioPortC, 7, gpioModePushPull, 1);		// SPI1 MISO
    //GPIO_PinModeSet(gpioPortC, 8, gpioModePushPull, 1);		// SPI1 SCK
    //GPIO_PinModeSet(gpioPortC, 9, gpioModePushPull, 1);		// SPI1 CS ADXL362
    //GPIO_PinModeSet(gpioPortC, 10, gpioModePushPull, 1);		// I2C1 SDA
    //GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, 1);		// I2C1 SCL

    //GPIO_PinModeSet(gpioPortF, 0, gpioModePushPull, 0);		// SWCLK
    //GPIO_PinModeSet(gpioPortF, 1, gpioModePushPull, 0);		// SWDIO
    //GPIO_PinModeSet(gpioPortF, 2, gpioModePushPull, 0);		// BUTTUN
    GPIO_PinModeSet(gpioPortF, 3, gpioModeInput, 0);			// LTC2942 AL/LC
    GPIO_PinModeSet(gpioPortF, 4, gpioModeInput, 0);			// ADPD188 INT
    GPIO_PinModeSet(gpioPortF, 5, gpioModeInput, 0);			// TEMP INT
    //GPIO_PinModeSet(gpioPortF, 6, gpioModePushPull, 0);		// BUZZER
    //GPIO_PinModeSet(gpioPortF, 7, gpioModePushPull, 0);		// 5V ON



    RMU_ResetCauseClear();

    SLEEP_ForceSleepInEM4();
}


