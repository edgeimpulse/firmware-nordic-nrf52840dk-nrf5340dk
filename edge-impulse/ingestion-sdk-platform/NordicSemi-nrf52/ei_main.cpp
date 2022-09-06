#include <stdio.h>

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_main.h"
#include "ei_run_impulse.h"
#include "ei_device_nordic_nrf52.h"
#include "ei_inertialsensor.h"
#include "ei_microphone.h"
#include "ei_at_handlers.h"
#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>
#include <device.h>
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
