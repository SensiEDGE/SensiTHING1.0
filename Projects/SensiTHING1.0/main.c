/***************************************************************************//**
 * @file   main.c
 * @brief  SensiTHING1.0 demo project.
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
 *******************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 ******************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "Application.h"


#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS     1
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
        .config_flags = 0,
        .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
        .bluetooth.max_connections = MAX_CONNECTIONS,
        .bluetooth.heap = bluetooth_stack_heap,
        .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
        .bluetooth.sleep_clock_accuracy = 100, // ppm
        .gattdb = &bg_gattdb_data,
        .ota.flags = 0,
        .ota.device_name_len = 3,
        .ota.device_name_ptr = "OTA",
        .max_timers = 8,   // The maximum number of concurrent timers. Up to 16 concurrent timers can be configured.
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
        .pa.config_enable = 1, // Enable high power PA
        .pa.input = GECKO_RADIO_PA_INPUT_VBAT,// Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
    };

/**
 * @brief      Main function
 */
int main(void)
{
    // Initialize device
    initMcu();

    // Initialize application
    Application_init();

    // Initialize stack
    gecko_init(&config);

    while(1) {
        // Event pointer for handling events
        struct gecko_cmd_packet* evt;

        // Check for stack event
        evt = gecko_wait_event();

        // Handle events
        Application_handleEvents(evt);
    }

    return 0;
}

