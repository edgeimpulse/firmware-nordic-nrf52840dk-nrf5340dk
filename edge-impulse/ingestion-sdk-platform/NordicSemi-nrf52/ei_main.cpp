/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_main.h"
#include "ei_run_impulse.h"
#include "ei_device_nordic_nrf52.h"
#include "ei_inertialsensor.h"
#include "ei_microphone.h"
#include "ei_at_handlers.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <soc.h>
#include "ble_nus.h"

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 5

static ATServer *at;

void ei_init(void)
{
    ei_printf("Hello from Edge Impulse Device SDK.\r\n"
              "Compiled on %s %s\r\n", __DATE__, __TIME__);

    ble_nus_init();

    /* Setup the inertial sensor */
    if(ei_inertial_init() == false) {
        ei_printf("Inerial sensor communication error occured\r\n");
    }

    /* Setup the microphone sensor */
    if(ei_microphone_init() == false) {
        ei_printf("Microphone intitialization failed\r\n");
    }

    at = ei_at_init();
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();

    EiDevice.set_state(eiStateFinished);
}

void ei_main(void)
{
    char data = uart_getchar();

    while(data != 0xFF) {
        at->handle(data);
        data = uart_getchar();
    }

    if(ei_ble_rcv_cmd_flag){
        ei_ble_rcv_cmd_flag = false;
        for(size_t i = 0; i < strlen(ei_ble_rcv_cmd_buffer); i++) {
            at->handle(ei_ble_rcv_cmd_buffer[i]);
        }
        at->handle('\r');
        memset(ei_ble_rcv_cmd_buffer, 0x00, sizeof(ei_ble_rcv_cmd_buffer));
    }

}
