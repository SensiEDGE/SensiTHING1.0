/***************************************************************************//**
 * @file   SeGpio.c
 * @brief  Functions and data related to SeGpio. This file is a wrapper for the 'em_gpio' file.
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

#include "gpiointerrupt.h"
#include "em_cmu.h"
#include "SeGpio.h"


/* Private variables. */

/* Interrupt initialization flag. */
static bool InterruptsInitFlag = false;


/* Public functions. */

/**
 * @brief      Configurate pin.
 * @param[in]  const SeGpio_pin_t *pin - Pointer to pin structure.
 * @param[in]  SeGpio_pinMode_t mode - Pin mode.
 * @param[in]  bool initState - Pin state after initialization for output mode. Or the pull-up/down direction for input mode.
 *             'true' - high state, 'false' - low state.
 * @retval     None.
 */
void SeGpio_configPin(const SeGpio_pin_t *pin, SeGpio_pinMode_t mode, bool initState)
{
    /* Configure GPIO pins */
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(pin->m_port, pin->m_pin, mode, initState);
}

/**
 * @brief      Configurate external interrupt pin.
 * @param[in]  const SeGpio_pin_t *pin - Pointer to pin structure.
 * @param[in]  SeGpio_pinMode_t mode - Pin mode.
 * @param[in]  bool pull - Pull direction. 'true' - pull up, 'false' - pull down.
 * @param[in]  SeGpio_intEdge_t edge - Interrupt edge.
 * @param[out] SeGpio_irqCallbackPtr_t callback - GPIO interrupt callback function pointer.
 * @retval     None.
 */
void SeGpio_configExtIntPin(const SeGpio_pin_t *pin, SeGpio_pinMode_t mode, bool pull, SeGpio_intEdge_t edge, SeGpio_irqCallbackPtr_t callback)
{
    if(!InterruptsInitFlag) {
        GPIOINT_Init();
        InterruptsInitFlag = true;
    }

    // Register callback functions
    GPIOINT_CallbackRegister(pin->m_pin, callback);

    bool risingEdge  = (edge == SE_GPIO_INT_MODE_RISING  || edge == SE_GPIO_INT_MODE_RISING_FALLING);
    bool fallingEdge = (edge == SE_GPIO_INT_MODE_FALLING || edge == SE_GPIO_INT_MODE_RISING_FALLING);

    GPIO_PinModeSet(pin->m_port, pin->m_pin, mode, pull);
    GPIO_ExtIntConfig(pin->m_port, pin->m_pin, pin->m_pin, risingEdge, fallingEdge,false);
}

/**
 * @brief      Enable/Disable external interrupt.
 * @param[in]  const SeGpio_pin_t *pin - Pointer to pin structure.
 * @param[in]  bool state - 'true' - enable interrupt, 'false' - disable
 * @retval     None
 */
void SeGpio_enableExtInterrupt(const SeGpio_pin_t *pin, bool state)
{
    if(state) {
        GPIO_IntEnable(1 << pin->m_pin);
    } else {
        GPIO_IntDisable(1 << pin->m_pin);
    }
}

/**
 * @brief      Set output pin state.
 * @param[in]  const SeGpio_pin_t *pin - Pointer to pin structure.
 * @param[in]  bool state - Pin state. 'true' - high state, 'false' - low state.
 * @retval     None.
 */
void SeGpio_setOutPin(const SeGpio_pin_t *pin, bool state)
{
    if(state) {
        GPIO_PinOutSet(pin->m_port, pin->m_pin);
    } else {
       GPIO_PinOutClear(pin->m_port, pin->m_pin);
    }
}

/**
 * @brief      Get input pin state.
 * @param[in]  const SeGpio_pin_t *pin - Pointer to pin structure.
 * @retval     bool state - Pin state. 'true' - high state, 'false' - low state.
 */
bool SeGpio_getInPin(const SeGpio_pin_t *pin)
{
  return (GPIO_PinInGet(pin->m_port, pin->m_pin) != 0);
}
