/***************************************************************************//**
 * @file   ApplicationTimer.h
 * @brief  This file contains the definition and enumeration of application software timer.
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

#ifndef _APPLICATION_TIMER_H_
#define _APPLICATION_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/* Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)
/* Stop timer. */
#define TIMER_STOP 0

/* Application soft timer enumeration. */
typedef enum {

    /* Application UI timer. */
    EVT_TIMER_UI_BUTTON_SHORT = 0,
    EVT_TIMER_UI_BUTTON_LONG,
    EVT_TIMER_UI_LEDS,

    /* Battery measurement timer. This is an auto-reload timer used for timing battery voltage measurements and sending a notification. */
    EVT_TIMER_BATTERY,

    /* Temperature and humidity measurement timer. This is an auto-reload timer used for timing temperature measurements and sending a notification. */
    EVT_TIMER_ENVIRONMENTAL,

    /* Accelerometer measurement timer. This is an auto-reload timer used for timing accelerometer measurements and sending a notification. */
    EVT_TIMER_ACCELEROMETER,

    /* GPIO ADC measurement timer. This is an auto-reload timer used for timing ADC measurements on chip AD5592R and sending a notification. */
    EVT_TIMER_GPIO_ADC,

	/* Turn off buzzer after some time */
    EVT_TIMER_GPIO_ADC_BUZZER_OFF,

} ApplicationTimer_t;


#endif /* _APPLICATION_TIMER_H_ */
