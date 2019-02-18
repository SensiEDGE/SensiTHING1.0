/***************************************************************************//**
 * @file   TimeUtils.c
 * @brief  Functions and data related to TimeUtils.
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

#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtcc.h"

#include "TimeUtils.h"


/* Public functions. */

/**
 * @brief      Config module to 1 ms ticks.
 * @retval     None.
 */
void TimeUtils_init(void)
{
    RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
    rtccInit.enable                = true;
    rtccInit.debugRun              = false;
    rtccInit.precntWrapOnCCV0      = false;
    rtccInit.cntWrapOnCCV1         = false;
    rtccInit.prescMode             = rtccCntTickPresc;
    rtccInit.presc                 = rtccCntPresc_8;
    rtccInit.enaOSCFailDetect      = false;
    rtccInit.cntMode               = rtccCntModeNormal;
    RTCC_Init(&rtccInit);

    CMU_ClockEnable(cmuClock_RTCC, true);
}

/**
 * @brief      Get milliseconds.
 * @retval     uint32_t - Time in ms.
 */
uint32_t TimeUtils_getMs(void)
{
    return (RTCC_CounterGet() * 1000) / 4096;
}

/**
 * @brief      Delays number of tiks (typically 1 ms).
 * @param[in]  uint32_t dlyTicks - Number of ticks to delay.
 * @retval     None.
 */
void TimeUtils_delay(uint32_t dlyTicks)
{
  uint32_t l_curTicks;

  l_curTicks = TimeUtils_getMs();
  while ((TimeUtils_getMs() - l_curTicks) < dlyTicks) ;
}
