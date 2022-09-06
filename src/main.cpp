#include <zephyr.h>
//#include "ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

#include <sys/printk.h>
#include "ei_device_nordic_nrf52.h"
#include "ei_zephyr_flash_commands.h"
#include "ei_main.h"


int main(void)
{
    /* This is needed so that output of printf
       is output immediately without buffering
    */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Initialize board uart */
    if(uart_init() != 0){
        ei_printf("Init uart on board error occured\r\n");
    }

    /** Initialize development board LEDs */
    if(BOARD_ledInit() != 0){
        ei_printf("Init LEDs on board error occured\r\n");
    }

    /* Initialize Zephyr flash device */
    create_flash_device();

    /* Initialize Edge Impuls sensors and commands */
    ei_init();

    while(1){
        ei_main();
    }
}
