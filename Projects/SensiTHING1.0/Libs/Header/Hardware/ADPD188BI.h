/***************************************************************************//**
 * @file   ADPD188BI.h
 * @brief  Header file of ADPD188BI Driver.
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

#ifndef _ADPD188BI_H_
#define _ADPD188BI_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Porting.h"
#include "SeI2c.h"
#include "SeGpio.h"

/* Maximum size of hardware FIFO buffer. */
#define ADPD188BI_FIFO_BUF_MAX_SIZE     128

/* ADPD188BI registers. */
typedef enum {
    ADPD188BI_REG_STATUS            = 0x00,
    ADPD188BI_REG_INT_MASK          = 0x01,
    ADPD188BI_REG_GPIO_DRV          = 0x02,
    ADPD188BI_REG_BG_STATUS         = 0x04,
    ADPD188BI_REG_FIFO_THRESH       = 0x06,
    ADPD188BI_REG_DEVID             = 0x08,
    ADPD188BI_REG_I2CS_ID           = 0x09,
    ADPD188BI_REG_CLK_RATIO         = 0x0A,
    ADPD188BI_REG_GPIO_CTRL         = 0x0B,
    ADPD188BI_REG_SLAVE_ADDRESS_KEY = 0x0D,
    ADPD188BI_REG_SW_RESET          = 0x0F,
    ADPD188BI_REG_MODE              = 0x10,
    ADPD188BI_REG_SLOT_EN           = 0x11,
    ADPD188BI_REG_FSAMPLE           = 0x12,
    ADPD188BI_REG_PD_LED_SELECT     = 0x14,
    ADPD188BI_REG_NUM_AVG           = 0x15,
    ADPD188BI_REG_BG_MEAS_A         = 0x16,
    ADPD188BI_REG_SLOTA_CH1_OFFSET  = 0x18,
    ADPD188BI_REG_SLOTA_CH2_OFFSET  = 0x19,
    ADPD188BI_REG_SLOTA_CH3_OFFSET  = 0x1A,
    ADPD188BI_REG_SLOTA_CH4_OFFSET  = 0x1B,
    ADPD188BI_REG_BG_MEAS_B         = 0x1C,
    ADPD188BI_REG_SLOTB_CH1_OFFSET  = 0x1E,
    ADPD188BI_REG_SLOTB_CH2_OFFSET  = 0x1F,
    ADPD188BI_REG_SLOTB_CH3_OFFSET  = 0x20,
    ADPD188BI_REG_SLOTB_CH4_OFFSET  = 0x21,
    ADPD188BI_REG_ILED3_COARSE      = 0x22,
    ADPD188BI_REG_ILED1_COARSE      = 0x23,
    ADPD188BI_REG_ILED2_COARSE      = 0x24,
    ADPD188BI_REG_ILED_FINE         = 0x25,
    ADPD188BI_REG_SLOTA_LED_PULSE   = 0x30,
    ADPD188BI_REG_SLOTA_NUMPULSES   = 0x31,
    ADPD188BI_REG_LED_DISABLE       = 0x34,
    ADPD188BI_REG_SLOTB_LED_PULSE   = 0x35,
    ADPD188BI_REG_SLOTB_NUMPULSES   = 0x36,
    ADPD188BI_REG_ALT_PWR_DN        = 0x37,
    ADPD188BI_REG_EXT_SYNC_STARTUP  = 0x38,
    ADPD188BI_REG_SLOTA_AFE_WINDOW  = 0x39,
    ADPD188BI_REG_SLOTB_AFE_WINDOW  = 0x3B,
    ADPD188BI_REG_AFE_PWR_CFG1      = 0x3C,
    ADPD188BI_REG_SLOTA_FLOAT_LED   = 0x3E,
    ADPD188BI_REG_SLOTB_FLOAT_LED   = 0x3F,
    ADPD188BI_REG_SLOTA_TIA_CFG     = 0x42,
    ADPD188BI_REG_SLOTA_AFE_CFG     = 0x43,
    ADPD188BI_REG_SLOTB_TIA_CFG     = 0x44,
    ADPD188BI_REG_SLOTB_AFE_CFG     = 0x45,
    ADPD188BI_REG_SAMPLE_CLK        = 0x4B,
    ADPD188BI_REG_CLK32M_ADJUST     = 0x4D,
    ADPD188BI_REG_EXT_SYNC_SEL      = 0x4F,
    ADPD188BI_REG_CLK32M_CAL_EN     = 0x50,
    ADPD188BI_REG_AFE_PWR_CFG2      = 0x54,
    ADPD188BI_REG_TIA_INDEP_GAIN    = 0x55,
    ADPD188BI_REG_MATH              = 0x58,
    ADPD188BI_REG_FLT_CONFIG_B      = 0x59,
    ADPD188BI_REG_FLT_LED_FIRE      = 0x5A,
    ADPD188BI_REG_FLT_CONFIG_A      = 0x5E,
    ADPD188BI_REG_DATA_ACCESS_CTL   = 0x5F,
    ADPD188BI_REG_FIFO_ACCESS       = 0x60,
    ADPD188BI_REG_SLOTA_PD1_16BIT   = 0x64,
    ADPD188BI_REG_SLOTA_PD2_16BIT   = 0x65,
    ADPD188BI_REG_SLOTA_PD3_16BIT   = 0x66,
    ADPD188BI_REG_SLOTA_PD4_16BIT   = 0x67,
    ADPD188BI_REG_SLOTB_PD1_16BIT   = 0x68,
    ADPD188BI_REG_SLOTB_PD2_16BIT   = 0x69,
    ADPD188BI_REG_SLOTB_PD3_16BIT   = 0x6A,
    ADPD188BI_REG_SLOTB_PD4_16BIT   = 0x6B,
    ADPD188BI_REG_A_PD1_LOW         = 0x70,
    ADPD188BI_REG_A_PD2_LOW         = 0x71,
    ADPD188BI_REG_A_PD3_LOW         = 0x72,
    ADPD188BI_REG_A_PD4_LOW         = 0x73,
    ADPD188BI_REG_A_PD1_HIGH        = 0x74,
    ADPD188BI_REG_A_PD2_HIGH        = 0x75,
    ADPD188BI_REG_A_PD3_HIGH        = 0x76,
    ADPD188BI_REG_A_PD4_HIGH        = 0x77,
    ADPD188BI_REG_B_PD1_LOW         = 0x78,
    ADPD188BI_REG_B_PD2_LOW         = 0x79,
    ADPD188BI_REG_B_PD3_LOW         = 0x7A,
    ADPD188BI_REG_B_PD4_LOW         = 0x7B,
    ADPD188BI_REG_B_PD1_HIGH        = 0x7C,
    ADPD188BI_REG_B_PD2_HIGH        = 0x7D,
    ADPD188BI_REG_B_PD3_HIGH        = 0x7E,
    ADPD188BI_REG_B_PD4_HIGH        = 0x7F,
} ADPD188BI_registers_t;

/* Time slot enumeration. */
typedef enum {
    ADPD188BI_SLOT_A    = 0,
    ADPD188BI_SLOT_B    = 1,
} ADPD188BI_slot_t;

/* Channels enumeration. */
typedef enum {
    ADPD188BI_CH_1      = 0,
    ADPD188BI_CH_2      = 1,
    ADPD188BI_CH_3      = 2,
    ADPD188BI_CH_4      = 3,
} ADPD188BI_channel_t;

/* GPIO pins enumeration. */
typedef enum {
    ADPD188BI_GPIO0     = 0,
    ADPD188BI_GPIO1     = 1,
} ADPD188BI_gpio_t;

/* Photodiodes connection configuration. */
typedef enum {
    ADPD188BI_PD_FLOATING                 = 0,  /* Inputs are floating in Time Slot */
    ADPD188BI_PD_P1P2CH1_E1E2CH2          = 1,  /* PDET1 and PDET2 are connected to Channel 1; EXT_IN1 and
                                                   EXT_IN2 are connected to Channel 2 during Time Slot */
    ADPD188BI_PD_E1CH1_E2CH2_P1CH3_P2CH4  = 5,  /* EXT_IN1 is connected to Channel 1, EXT_IN2 is
                                                   connected to Channel 2, PDET1 is connected to Channel 3,
                                                   and PDET2 is connected to Channel 4 during Time Slot */
} ADPD188BI_pdConfig_t;

/* LED connection configuration.  These determine which LED is associated with Time Slot. */
typedef enum {
    ADPD188BI_LED_PD_TO_AFE = 0,    /* Pulse photodiode connection to AFE. Float mode and pulse connect mode enable.*/
    ADPD188BI_LED_LEDX1     = 1,    /* LEDX1 pulses during Time Slot. */
    ADPD188BI_LED_LEDX2     = 2,    /* LEDX2 pulses during Time Slot. */
    ADPD188BI_LED_LEDX3     = 3,    /* LEDX3 pulses during Time Slot. */

} ADPD188BI_ledConfig_t;

/* Slots FIFO config enumeration. */
typedef enum {
    ADPD188BI_NO_FIFO       = 0,    /* No data to FIFO. */
    ADPD188BI_SUM_4CH_16    = 1,    /* 16-bit sum of all four channels. */
    ADPD188BI_SUM_4CH_32    = 2,    /* 32-bit sum of all four channels. */
    ADPD188BI_4CH_16        = 4,    /* Four channels of 16-bit sample data for Time Slot B. */
    ADPD188BI_4CH_32        = 6,    /* Four channels of 32-bit extended sample data for Time Slot B. */
} ADPD188BI_slotFifoConfig_t;

/* Averaging factor. Specifies the averaging factor, which is the number of consecutive samples that
   is summed and averaged after the ADC. */
typedef enum {
    ADPD188BI_NUM_AVG_1     = 0,    /* 1. */
    ADPD188BI_NUM_AVG_2     = 1,    /* 2. */
    ADPD188BI_NUM_AVG_4     = 2,    /* 4. */
    ADPD188BI_NUM_AVG_8     = 3,    /* 8. */
    ADPD188BI_NUM_AVG_16    = 4,    /* 16. */
    ADPD188BI_NUM_AVG_32    = 5,    /* 32. */
    ADPD188BI_NUM_AVG_64    = 6,    /* 64. */
    ADPD188BI_NUM_AVG_128   = 7,    /* 128. */
} ADPD188BI_averageFactor_t;

/* GPIO configuration. */
typedef enum {
    ADPD188BI_GPIO_ADPD103_MODE     = 0x00, /* GPIO1 is backward compatible to the ADPD103 PDSO pin functionality. */
    ADPD188BI_GPIO_INTERRUPT        = 0x01, /* Interrupt function provided on GPIO1, as defined in Register 0x01. */
    ADPD188BI_GPIO_START_END_SLOT   = 0x02, /* Asserts at the start of the first time slot, deasserts at end of last time slot. */
    ADPD188BI_GPIO_SLOT_A           = 0x05, /* Time Slot A pulse output. */
    ADPD188BI_GPIO_SLOT_B           = 0x06, /* Time Slot B pulse output. */
    ADPD188BI_GPIO_SLOT_AB          = 0x07, /* Pulse output of both time slots. */
    ADPD188BI_GPIO_CYCLE_SLOT_A     = 0x0C, /* Output data cycle occurred for Time Slot A. */
    ADPD188BI_GPIO_CYCLE_SLOT_B     = 0x0D, /* Output data cycle occurred for Time Slot B. */
    ADPD188BI_GPIO_CYCLE_SLOT_AB    = 0x0E, /* Output data cycle occurred.  */
    ADPD188BI_GPIO_EVERY_SAMPLE     = 0x0F, /* Toggles on every sample, which provides a signal at half the sampling rate. */
    ADPD188BI_GPIO_LOGIC0           = 0x10, /* Output = 0. */
    ADPD188BI_GPIO_LOGIC1           = 0x11, /* Output = 1. */
    ADPD188BI_GPIO_32KHZ            = 0x13, /* 32 kHz oscillator output. */
    ADPD188BI_GPIO0_DISABLE         = 0xFF, /* Disable pin. Only for GPIO0. */
} ADPD188BI_gpioConfig_t;

/* Internal state machine operation mode. Writes to register ADPD188BI_REG_MODE. */
typedef enum {
    ADPD188BI_STANDBY   = 0x00,     /* Ultralow power mode. No data collection all register values are retained. */
    ADPD188BI_PROGRAM   = 0x01,     /* Safe mode for programming registers. No data collection device is fully
                                       powered in this mode. */
    ADPD188BI_NORMAL    = 0x02,     /* Leds are pulsed and photodiodes are sampled. Standard data collection
                                       device power is cycled by internal state machine. */
} ADPD188BI_stateMachineMode_t;

/* Structure that contains the configuration information for ADPD188BI. */
typedef struct {
    SeI2c_t             *m_hi2c;
    uint8_t             m_address;      /* I2C address. */
    uint8_t             m_buffer[ADPD188BI_FIFO_BUF_MAX_SIZE];
} ADPD188BI_t;


/* Initialize ADPD188BI. */
bool ADPD188BI_init(ADPD188BI_t *handle, SeI2c_t *hi2c, uint8_t address);

/* Software reset. */
bool ADPD188BI_softReset(ADPD188BI_t *handle);

/* Set ADPD188BI operating mode. */
bool ADPD188BI_setOperationMode(ADPD188BI_t *handle, ADPD188BI_stateMachineMode_t mode);

/* LEDs configuration. */
bool ADPD188BI_setLedConfig(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_ledConfig_t conf);

/* Photodiods configuration. */
bool ADPD188BI_setPdConfig(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_pdConfig_t conf);

/* GPIO configuration. */
bool ADPD188BI_setGpioConfig(ADPD188BI_t *handle, ADPD188BI_gpio_t pin, ADPD188BI_gpioConfig_t conf);

/* Set slot FIFO configuration. */
bool ADPD188BI_setSlotsFifoConfig(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_slotFifoConfig_t conf);

/* Enable/disable time slot. */
bool ADPD188BI_enableSlots(ADPD188BI_t *handle, ADPD188BI_slot_t slot, bool state);

/* Set averaging factor. */
bool ADPD188BI_setAdcAvgFactor(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_averageFactor_t avg);

/* Enable/disable FIFO interrupt. */
bool ADPD188BI_enableFiFoInterrupts(ADPD188BI_t *handle, uint16_t threshold);

/* Read measurement data from register. */
bool ADPD188BI_readData16FromRegister(ADPD188BI_t *handle, ADPD188BI_slot_t slot, ADPD188BI_channel_t channel, uint16_t *value);

/* Get the number of bytes in FIFO. */
bool ADPD188BI_getFifoBytes(ADPD188BI_t *handle, uint16_t *bytes);

/* Read the data from FIFO buffer in 16-bit format. */
bool ADPD188BI_readData16FromFifo(ADPD188BI_t *handle, uint16_t *buffer, uint16_t buffSize, uint16_t samples);

/* Clear FIFO buffer. */
bool ADPD188BI_clearFifo(ADPD188BI_t *handle);

/* Work with raw data. */
bool ADPD188BI_setRegisterValue(ADPD188BI_t *handle, ADPD188BI_registers_t reg, uint16_t value);
bool ADPD188BI_getRegisterValue(ADPD188BI_t *handle, ADPD188BI_registers_t reg, uint16_t *value);


#ifdef __cplusplus
}
#endif

#endif /* _ADPD188BI_H_ */
