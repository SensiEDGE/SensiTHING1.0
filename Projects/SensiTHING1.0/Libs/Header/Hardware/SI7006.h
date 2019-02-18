/***************************************************************************//**
 * @file   SI7006.h
 * @brief  Header file of SI7006 Driver.
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

#ifndef _SI7006_H_
#define _SI7006_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Porting.h"
#include "SeI2c.h"


/* Structure that contains the configuration information for SI7006. */
typedef struct {
    SeI2c_t     *m_hi2c;
    uint8_t     m_address;
} SI7006_t;


/* Initialize SI7006. */
bool SI7006_init(SI7006_t *handle, SeI2c_t *hi2c, uint8_t address);

/* Read humidity. */
bool SI7006_getRh(SI7006_t *handle, float *rh);

/* Read temperature. */
bool SI7006_getTemp(SI7006_t *handle, float *temp);

/* Read temperature after reading humidity. This function does not perform a measurement but returns the
   temperature value measured during the relative humidity measurement.*/
bool SI7006_getTempAfterRh(SI7006_t *handle, float *temp);

/* Enable/disable heater. */
bool SI7006_heater(SI7006_t *handle, bool state);

/* Set heater power level. */
bool SI7006_heaterLevel(SI7006_t *handle, uint8_t level);


#ifdef __cplusplus
}
#endif

#endif /* _SI7006_H_ */
