#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

#include "board_features.h"
#include "hal-config-board.h"
#include "hal-config-app-common.h"


/* UART port used for trace debug information. */
#define BSP_SERIAL_APP_PORT             (HAL_SERIAL_PORT_USART0)

#define BSP_SERIAL_APP_RX_PIN           (1U)
#define BSP_SERIAL_APP_RX_PORT          (gpioPortA)
#define BSP_SERIAL_APP_RX_LOC           (0U)

#define BSP_SERIAL_APP_TX_PIN           (0U)
#define BSP_SERIAL_APP_TX_PORT          (gpioPortA)
#define BSP_SERIAL_APP_TX_LOC           (0U)


/* I2C port */
#define BSP_I2C0_PORT                   I2C0

#define BSP_I2C0_SDA_LOC                (8U)	//PB13
#define BSP_I2C0_SCL_LOC                (3U)	//PA4


#define BSP_I2C1_PORT                   I2C1

#define BSP_I2C1_SDA_LOC                (19U)	//PC10
#define BSP_I2C1_SCL_LOC                (19U)	//PC11

/* SPI port */
#define BSP_SPI1_PORT                   USART1

#define BSP_SPI1_MOSI_LOC               (11U)	//PC6
#define BSP_SPI1_MIS0_LOC               (11U)	//PC7
#define BSP_SPI1_SCK_LOC                (11U)	//PC8


/* ADC */
#define BSP_ADC_PORT                    ADC0
#define BSP_ADC_CHARGING_CH             adcPosSelAPORT3YCH5    // PD13

/* Low energy timer pin location. Used for buzzer. */
#define BSP_LETIM_PORT                  LETIMER0
#define BSP_LETIM_CH0_LOC               (30U)	//PF6

/* GPIO */
/* ADT7420 */
#define BSP_ADT7420_INT_PIN             (5U)
#define BSP_ADT7420_INT_PORT            (gpioPortF)

#define BSP_ADPD188BI_GPIO0_PIN         (4U)
#define BSP_ADPD188BI_GPIO0_PORT        (gpioPortF)

#define BSP_ADPD188BI_5V_PIN         	(7U)
#define BSP_ADPD188BI_5V_PORT        	(gpioPortF)

/* ADXL362 */
#define BSP_ADXL362_CS_PIN              (9U)
#define BSP_ADXL362_CS_PORT             (gpioPortC)

#define BSP_ADXL362_INT1_PIN            (3U)
#define BSP_ADXL362_INT1_PORT           (gpioPortA)

/* AD5592R */
#define BSP_AD5592R_CS_PIN              (11U)
#define BSP_AD5592R_CS_PORT             (gpioPortB)

#define BSP_AD5592R_RESET_PIN           (5U)
#define BSP_AD5592R_RESET_PORT          (gpioPortA)

/* Button */
#define BSP_BUTTON_PIN                  (2U)
#define BSP_BUTTON_PORT                 (gpioPortF)

/* W25Q80 */
#define BSP_W25Q80_CS_PIN               (2U)
#define BSP_W25Q80_CS_PORT              (gpioPortA)

/* EM4 Wake Up Level for EM4WU0 Pin (PF2) */
/* A bitmask containing the bitwise logic OR of which GPIO pin(s) to enable. */
#define BSP_WAKEUP_LINE                  GPIO_EXTILEVEL_EM4WU0
/* A bitmask containing the bitwise logic OR of GPIO pin(s) wake-up polarity. */
#define BSP_WAKEUP_POLARITY              _GPIO_EXTILEVEL_EM4WU0_DEFAULT



/* I2C addresses */
#define BSP_ADT7420_I2C_ADR             0x48
#define BSP_SI7006_I2C_ADR              0x40
#define BSP_ADPD188BI_I2C_ADR           0x64
#define BSP_LTC2942_I2C_ADR				0x64

#endif /* HAL_CONFIG_H */
