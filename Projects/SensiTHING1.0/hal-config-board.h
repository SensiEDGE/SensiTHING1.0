#ifndef HAL_CONFIG_BOARD_H
#define HAL_CONFIG_BOARD_H

#include "em_device.h"
#include "hal-config-types.h"

// $[CMU]
#define BSP_CLK_HFXO_PRESENT                          (1)
#define BSP_CLK_HFXO_FREQ                             (38400000)
#define BSP_CLK_HFXO_INIT                              CMU_HFXOINIT_DEFAULT
#define BSP_CLK_HFXO_CTUNE                            (346)
#define BSP_CLK_HFXO_CTUNE_TOKEN                      (0)

#define BSP_CLK_LFXO_PRESENT                          (1)
#define BSP_CLK_LFXO_FREQ                             (32768)
#define BSP_CLK_LFXO_INIT                              CMU_LFXOINIT_DEFAULT
// [CMU]$

// $[DCDC]
#define BSP_DCDC_PRESENT                              (1)
#define BSP_DCDC_INIT                                  EMU_DCDCINIT_DEFAULT
// [DCDC]$

// $[UARTNCP]
#define BSP_UARTNCP_USART_PORT                        (HAL_SERIAL_PORT_USART0)
#define BSP_UARTNCP_CTS_PIN                           (2)
#define BSP_UARTNCP_CTS_PORT                          (gpioPortA)
#define BSP_UARTNCP_CTS_LOC                           (30)

#define BSP_UARTNCP_RX_PIN                            (1)
#define BSP_UARTNCP_RX_PORT                           (gpioPortA)
#define BSP_UARTNCP_RX_LOC                            (0)

#define BSP_UARTNCP_TX_PIN                            (0)
#define BSP_UARTNCP_TX_PORT                           (gpioPortA)
#define BSP_UARTNCP_TX_LOC                            (0)

#define BSP_UARTNCP_RTS_PIN                           (3)
#define BSP_UARTNCP_RTS_PORT                          (gpioPortA)
#define BSP_UARTNCP_RTS_LOC                           (30)
// [UARTNCP]$

// $[USART0]
#define BSP_UARTNCP_USART_PORT                        (HAL_SERIAL_PORT_USART0)
#define BSP_USART0_CTS_PIN                            (2)
#define BSP_USART0_CTS_PORT                           (gpioPortA)
#define BSP_USART0_CTS_LOC                            (30)

#define BSP_USART0_RTS_PIN                            (3)
#define BSP_USART0_RTS_PORT                           (gpioPortA)
#define BSP_USART0_RTS_LOC                            (30)

#define BSP_USART0_RX_PIN                             (1)
#define BSP_USART0_RX_PORT                            (gpioPortA)
#define BSP_USART0_RX_LOC                             (0)

#define BSP_USART0_TX_PIN                             (0)
#define BSP_USART0_TX_PORT                            (gpioPortA)
#define BSP_USART0_TX_LOC                             (0)
// [USART0]$
// Note: These might be necessary for the selected mcu or board as well.
#define HAL_PA_ENABLE                                 (1)
// #define FEATURE_PA_HIGH_POWER

#endif /* HAL_CONFIG_BOARD_H */