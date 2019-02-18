/***************************************************************************//**
 * @file   SeGpio.h
 * @brief  SeGpio header file. This file is a wrapper for the 'em_gpio' file.
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

#ifndef _SE_GPIO_H_
#define _SE_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "em_gpio.h"

#include "Porting.h"


/* GPIO pin modes. */
typedef enum {
    SE_GPIO_MODE_INPUT              = gpioModeInput,            /* Input. */
    SE_GPIO_MODE_INPUT_PULL_FILTER  = gpioModeInputPullFilter,  /* Input enabled with filter. */
    SE_GPIO_MODE_PUSH_PULL          = gpioModePushPull,         /* Push-pull output. */
	SE_GPIO_MODE_OD          		= gpioModeWiredAnd,			/* Open-drain output. */
} SeGpio_pinMode_t;

/* External interrupt mode. */
typedef enum {
    SE_GPIO_INT_MODE_RISING,            /* Rising edge trigger detection. */
    SE_GPIO_INT_MODE_FALLING,           /* Falling edge trigger detection. */
    SE_GPIO_INT_MODE_RISING_FALLING,    /* Rising and Falling edge trigger detection. */
} SeGpio_intEdge_t;

/**
 * @brief      GPIO interrupt callback function pointer.
 * @param[in]  uint8_t pin - The pin index the callback function is invoked for.
 * @retval     None.
 */
typedef void (*SeGpio_irqCallbackPtr_t)(uint8_t pin);

/* Pin definition structure */
typedef struct {
    GPIO_Port_TypeDef   m_port;
    uint8_t             m_pin;
} SeGpio_pin_t;


/* Configurate pin. */
void SeGpio_configPin(const SeGpio_pin_t *pin, SeGpio_pinMode_t mode, bool initState);

/* Configurate external interrupt pin. */
void SeGpio_configExtIntPin(const SeGpio_pin_t *pin, SeGpio_pinMode_t mode, bool pull, SeGpio_intEdge_t edge, SeGpio_irqCallbackPtr_t callback);

/* Enable/Disable external interrupt. */
void SeGpio_enableExtInterrupt(const SeGpio_pin_t *pin, bool state);

/* Set output pin state. */
void SeGpio_setOutPin(const SeGpio_pin_t *pin, bool state);

/* Get input pin state. */
bool SeGpio_getInPin(const SeGpio_pin_t *pin);


#ifdef __cplusplus
}
#endif

#endif /* _SE_GPIO_H_ */
