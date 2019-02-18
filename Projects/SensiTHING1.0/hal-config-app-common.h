#ifndef HAL_CONFIG_APP_COMMON_H
#define HAL_CONFIG_APP_COMMON_H

#include "em_device.h"
#include "hal-config-types.h"

#if defined(FEATURE_IOEXPANDER)
#include "hal-config-ioexp.h"
#endif

#if defined(FEATURE_FEM)
#include "hal-config-fem.h"
#endif

#ifdef BSP_CLK_LFXO_CTUNE
#undef BSP_CLK_LFXO_CTUNE
#endif
#define BSP_CLK_LFXO_CTUNE                            (32)

#if (HAL_PA_ENABLE)
#define HAL_PA_RAMP                                   (10)
#define HAL_PA_2P4_LOWPOWER                           (0)
#define HAL_PA_POWER                                  (252)
#define HAL_PA_CURVE_HEADER                            "pa_curves_efr32.h"
#endif

#ifdef FEATURE_PA_HIGH_POWER
#define HAL_PA_VOLTAGE                                (3300)
#else // FEATURE_PA_HIGH_POWER
#define HAL_PA_VOLTAGE                                (1800)
#endif // FEATURE_PA_HIGH_POWER

#define HAL_VCOM_ENABLE                   (0)
#define HAL_I2CSENSOR_ENABLE              (0)
#define HAL_SPIDISPLAY_ENABLE             (0)

#endif /* HAL_CONFIG_APP_COMMON_H */