/***************************************************************************//**
 * @file   LTC2942.h
 * @brief  Header file of LTC2942 Driver.
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

#ifndef _LTC2942_H_
#define _LTC2942_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Porting.h"
#include "SeI2c.h"



// LTC2942 register definitions
#define LTC2942_REG_STATUS              (uint8_t)0x00 // (A) Status
#define LTC2942_REG_CONTROL             (uint8_t)0x01 // (B) Control
#define LTC2942_REG_AC_H                (uint8_t)0x02 // (C) Accumulated charge MSB
#define LTC2942_REG_AC_L                (uint8_t)0x03 // (D) Accumulated charge LSB
#define LTC2942_REG_CTH_H               (uint8_t)0x04 // (E) Charge threshold high MSB
#define LTC2942_REG_CTH_L               (uint8_t)0x05 // (F) Charge threshold high LSB
#define LTC2942_REG_CTL_H               (uint8_t)0x06 // (G) Charge threshold low MSB
#define LTC2942_REG_CTL_L               (uint8_t)0x07 // (H) Charge threshold low LSB
#define LTC2942_REG_VOL_H               (uint8_t)0x08 // (I) Voltage MSB
#define LTC2942_REG_VOL_L               (uint8_t)0x09 // (J) Voltage LSB
#define LTC2942_REG_VOLT_H              (uint8_t)0x0A // (K) Voltage threshold high
#define LTC2942_REG_VOLT_L              (uint8_t)0x0B // (L) Voltage threshold low
#define LTC2942_REG_TEMP_H              (uint8_t)0x0C // (M) Temperature MSB
#define LTC2942_REG_TEMP_L              (uint8_t)0x0D // (N) Temperature LSB
#define LTC2942_REG_TEMPT_H             (uint8_t)0x0E // (O) Temperature threshold high
#define LTC2942_REG_TEMPT_L             (uint8_t)0x0F // (P) Temperature threshold low

// LTC2942 status register bit definitions
#define LTC2942_STATUS_CHIPID           (uint8_t)0x80 // A[7] Chip identification (0: LTC2942-1, 0: LTC2941-1)
#define LTC2942_STATUS_AC_OVR           (uint8_t)0x20 // A[5] Accumulated charge overflow/underflow
#define LTC2942_STATUS_TEMP_ALRT        (uint8_t)0x10 // A[4] Temperature alert
#define LTC2942_STATUS_CHG_ALRT_H       (uint8_t)0x08 // A[3] Charge alert high
#define LTC2942_STATUS_CHG_ALRT_L       (uint8_t)0x04 // A[2] Charge alert low
#define LTC2942_STATUS_VOL_ALRT         (uint8_t)0x02 // A[1] Voltage alert
#define LTC2942_STATUS_UVLO_ALRT        (uint8_t)0x01 // A[0] Undervoltage lockout alert

// LTC2942 control register bit definitions
#define LTC2942_CTL_ADC_MSK             (uint8_t)0x3F // ADC mode bits [7:6]
#define LTC2942_CTL_PSCM_MSK            (uint8_t)0xC7 // Prescaler M bits [5:3]
#define LTC2942_CTL_ALCC_MSK            (uint8_t)0xF9 // AL/CC pin control [2:1]
#define LTC2942_CTL_SHUTDOWN            (uint8_t)0x01 // B[0] Shutdown


// LTC2942 ADC mode enumeration
enum {
	LTC2942_ADC_AUTO   = (uint8_t)0xC0, // Automatic mode
	LTC2942_ADC_M_VOL  = (uint8_t)0x80, // Manual voltage mode
	LTC2942_ADC_M_TEMP = (uint8_t)0x40, // Manual temperature mode
	LTC2942_ADC_SLEEP  = (uint8_t)0x00  // Sleep
};

// LTC2942 prescaler M enumeration
enum {
	LTC2942_PSCM_1   = (uint8_t)0x00,
	LTC2942_PSCM_2   = (uint8_t)0x08,
	LTC2942_PSCM_4   = (uint8_t)0x10,
	LTC2942_PSCM_8   = (uint8_t)0x18,
	LTC2942_PSCM_16  = (uint8_t)0x20,
	LTC2942_PSCM_32  = (uint8_t)0x28,
	LTC2942_PSCM_64  = (uint8_t)0x30,
	LTC2942_PSCM_128 = (uint8_t)0x38
};

// LTC2942 AL/CC pin mode enumeration
enum {
	LTC2942_ALCC_DISABLED = (uint8_t)0x00, // AL/CC pin disabled
	LTC2942_ALCC_CHG      = (uint8_t)0x02, // Charge complete mode
	LTC2942_ALCC_ALERT    = (uint8_t)0x04  // Alert mode
};

// State of analog section enumeration
enum {
	LTC2942_AN_DISABLED = (uint8_t)0x00,
	LTC2942_AN_ENABLED  = !LTC2942_AN_DISABLED
};


/* Structure that contains the configuration information for LTC2942. */
typedef struct {
    SeI2c_t     *m_hi2c;
    uint8_t     m_address;
} LTC2942_t;


/* Initialize LTC2942. */
bool LTC2942_init(LTC2942_t *handle, SeI2c_t *hi2c, uint8_t address);

/* Get voltage. */
bool LTC2942_getVoltage(LTC2942_t *handle, float *voltage);

bool LTC2942_setADCMode(LTC2942_t *handle, uint8_t mode);

bool LTC2942_setAnalog(LTC2942_t *handle, uint8_t state);

bool LTC2942_setPrescaler(LTC2942_t *handle, uint8_t psc);

bool LTC2942_setALCCMode(LTC2942_t *handle, uint8_t mode);

bool LTC2942_getAC(LTC2942_t *handle, uint16_t *ac);


#ifdef __cplusplus
}
#endif

#endif /* _LTC2942_H_ */
