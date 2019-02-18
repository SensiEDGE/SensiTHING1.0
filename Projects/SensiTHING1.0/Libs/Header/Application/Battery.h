/***************************************************************************//**
 * @file   Battery.h
 * @brief  Battery header file
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

#ifndef _BATTERY_H_
#define _BATTERY_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Porting.h"
#include "SeI2c.h"

/* Initialize battery module. */
void Battery_init(SeI2c_t *hi2c);

/* Get battery voltage. */
float Battery_getVoltage(void);

/* Is battery charging. */
bool Battery_isCharging(void);

/* Get battery capacity. */
float Battery_getCapacity(void);

/* Calculate the battery capacity using the voltage. */
float Battery_calcCapacity(float voltage);

/* Function that is called when the characteristic status is changed. */
void Battery_charStatusChange(uint8_t connection, uint16_t attribute, uint16_t flags);

/* Soft timer event handler. */
void Battery_timHandler(uint16_t timId);

/* Turns off the battery measurement. For example, when the close connection. */
void Battery_off(void);


#ifdef __cplusplus
}
#endif

#endif /* _BATTERY_H_ */
