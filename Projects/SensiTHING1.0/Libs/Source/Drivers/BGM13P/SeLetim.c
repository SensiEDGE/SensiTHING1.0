/***************************************************************************//**
 * @file   SeLetim.c
 * @brief  Functions and data related to SeLetim. This driver only supports one
 *         low energy timer channel, in PWM mode.
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

#include "SeLetim.h"


/* Private defines. */

/* Clock frequency of the timer. */
#define SE_LE_TIM_FRQ_CLK           (32768)


/* Public functions. */

/**
 * @brief      Initialize the timer according to the specified parameters.
 * @param[in]  SeLetim_t *htim - Pointer to a SeLetim_t structure that contains
 *             the configuration information for timer.
 * @param[in]  const SeLetim_init_t *init - Pointer to timer initialization structure.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool SeLetim_init(SeLetim_t *htim, const SeLetim_init_t *init)
{
    if(!htim || !init) {
        return false;
    }

    htim->m_port = init->m_port;

    CMU_Clock_TypeDef timClock;
    GPIO_Port_TypeDef l_ch0Port;
    uint16_t l_ch0Pin;

     /* Enable necessary clocks */
     CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
     CMU_ClockEnable(cmuClock_CORELE, true);
     CMU_ClockEnable(cmuClock_GPIO, true);

    // Select LETIMER peripheral clock
    if(false){

    }
    #if defined(LETIMER0)
    else if (init->m_port == LETIMER0) {
        timClock = cmuClock_LETIMER0;
        l_ch0Port = AF_LETIMER0_OUT0_PORT(init->m_ch0Loc);
        l_ch0Pin = AF_LETIMER0_OUT0_PIN(init->m_ch0Loc);
    }
    #endif
    else {
        // Timer is not defined
        return false;
    }

    CMU_ClockEnable(timClock, true);

    /* Pin is configured to Push-pull. */
    GPIO_PinModeSet(l_ch0Port, l_ch0Pin, gpioModePushPull, 0);


    /* Repetition values must be nonzero so that the outputs return switch between idle and active state */
    LETIMER_RepeatSet(htim->m_port, 0, 0x01);
    LETIMER_RepeatSet(htim->m_port, 1, 0x01);

    htim->m_port->ROUTELOC0 = (htim->m_port->ROUTELOC0 & ~_LETIMER_ROUTELOC0_OUT0LOC_MASK) | init->m_ch0Loc;
    htim->m_port->ROUTEPEN |= LETIMER_ROUTEPEN_OUT0PEN;


    /* Set configurations for LETIMER */
    const LETIMER_Init_TypeDef letimerInit =
    {
        .enable         = false,                  /* Start counting when init completed. */
        .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
        .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
        .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
        .out0Pol        = !init->m_polarity,      /* Idle value for output 0. */
        .out1Pol        = 0,                      /* Idle value for output 1. */
        .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
        .ufoa1          = letimerUFOANone,        /* PWM output on output 1*/
        .repMode        = letimerRepeatFree       /* Count until stopped */
    };

    /* Initialize LETIMER */
    LETIMER_Init(htim->m_port, &letimerInit);

    // LETimer is 16 bit long and oscillator frequency is 32768Hz.
    // PWM duty-cycle and frequency can be determined using these compare values
    // Shortly the first one determines frequency and second one determines duty-cycle.
    LETIMER_CompareSet(htim->m_port, 0, 0);
    LETIMER_CompareSet(htim->m_port, 1, 0);

    return true;
}

/**
 * @brief      Set timer configuration.
 * @param[in]  SeLetim_t *htim - Pointer to a SeLetim_t structure that contains
 *             the configuration information for timer.
 * @param[in]  uint16_t frequency - Target frequency, can have a value of up to
 *             half the clock frequency of the timer (16384 Hz).
 * @param[in]  uint8_t dutyRatio - Target duty ratio, can have a value 0 - 100;
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
void SeLetim_setConfig(SeLetim_t *htim, uint16_t frequency, uint8_t dutyRatio)
{
    if(htim) {
        if(frequency > SE_LE_TIM_FRQ_CLK/2) {
            frequency = SE_LE_TIM_FRQ_CLK/2;
        }

        if(dutyRatio > 100) {
            dutyRatio = 100;
        }

        uint16_t l_frq = SE_LE_TIM_FRQ_CLK/frequency;

        uint16_t l_duty = (l_frq * dutyRatio) / 100;

        if(dutyRatio != 0 && l_duty < 1) {
            l_duty = 1;
        }

        LETIMER_CompareSet(htim->m_port, 0, l_frq);
        LETIMER_CompareSet(htim->m_port, 1, l_duty);
    }
}

/**
 * @brief      Enable/disable timer.
 * @param[in]  SeLetim_t *htim - Pointer to a SeLetim_t structure that contains
 *             the configuration information for timer.
 * @param[in]  bool state - New state. 'true' - to enable counting, 'false' to disable.
 * @retval     None.
 */
void SeLetim_enable(SeLetim_t *htim, bool state)
{
    if(htim) {
        LETIMER_Enable(htim->m_port, state);
    }
}
