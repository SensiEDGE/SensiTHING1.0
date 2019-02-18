/***************************************************************************//**
 * @file   Accelerometer.h
 * @brief  Accelerometer header file
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

#ifndef _ACCELEROMETER_H_
#define _ACCELEROMETER_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Porting.h"
#include "SeSpi.h"


/* Initialize accelerometer module. */
void Accelerometer_init(SeSpi_t *hspi);

/* Function that is called when the characteristic status is changed. */
void Accelerometer_charStatusChange(uint8_t connection, uint16_t attribute, uint16_t flags);

/* Soft timer event handler. */
void Accelerometer_timHandler(uint16_t timId);

/* External interrupt event handler. */
void Accelerometer_extIntHandler(uint16_t signals);

/* Turns off the accelerometer. For example, when the close connection. */
void Accelerometer_off(void);


#ifdef __cplusplus
}
#endif

#endif /* _ACCELEROMETER_H_ */
