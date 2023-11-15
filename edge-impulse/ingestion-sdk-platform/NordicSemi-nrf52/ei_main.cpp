/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
