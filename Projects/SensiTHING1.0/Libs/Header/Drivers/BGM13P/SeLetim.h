/***************************************************************************//**
 * @file   SeLetim.h
 * @brief  SeLetim header file
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

#ifndef _SE_LETIM_H_
#define _SE_LETIM_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "em_letimer.h"
#include "em_gpio.h"

#include "Porting.h"


/* I2C driver instance initialization structure.
   This data structure contains a number of I2C configuration options
   required for driver instance initialization.
   This struct is passed to @ref SeLetim_init() when initializing a timer instance. */
typedef struct {
    LETIMER_TypeDef     *m_port;    /* Peripheral port. */
    uint8_t             m_ch0Loc;   /* Channel 0 pin location. */
    bool                m_polarity; /* Channel polarity. When 'true' idle value '0', pulse value '1'. */

} SeLetim_init_t;


/* Structure that contains the configuration information for low energy timer. */
typedef struct {
    LETIMER_TypeDef    *m_port;    /* Timer peripheral port. */
} SeLetim_t;


/* Initialize the timer according to the specified parameters. */
bool SeLetim_init(SeLetim_t *htim, const SeLetim_init_t *init);

/* Set configuration. */
void SeLetim_setConfig(SeLetim_t *htim, uint16_t frequency, uint8_t dutyRatio);

/* Enable/disable timer. */
void SeLetim_enable(SeLetim_t *htim, bool state);


#ifdef __cplusplus
}
#endif

#endif /* _SE_LETIM_H_ */
