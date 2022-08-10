#include <stdio.h>

#include "ei_device_nordic_nrf52.h"
#include "ei_zephyr_flash_commands.h"
#include "ei_classifier_porting.h"
#include "ei_main.h"
#include "ei_run_impulse.h"
#include "../../repl/at_cmds.h"

#include "ei_inertialsensor.h"
#include "ei_microphone.h"

/******/
#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>

#include <device.h>
#include <soc.h>
#include "ble_nus.h"
/*****/

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 5

#include "ble_nus.h"

/* help function for list files which is not currently implemented */
void list_files_help_fn(void (*data_fn)(char *))
{
    char no_files[] = "No files";
    data_fn(no_files);
}

void ei_init(void)
{
    ei_printf("Hello from Edge Impulse Device SDK.\r\n"
              "Compiled on %s %s\r\n", __DATE__, __TIME__);

    ble_nus_init();

    /* Setup the inertial sensor */
    if(ei_inertial_init() == false) {
        ei_printf("Inertial sensor communication error occured\r\n");
    }

    /* Setup the microphone sensor */
    if(ei_microphone_init() == false) {
        ei_printf("Microphone intitialization failed\r\n");
    }


    /* Intialize configuration */
    static ei_config_ctx_t config_ctx = { 0 };
    config_ctx.get_device_id = EiDevice.get_id_function();
    config_ctx.get_device_type = EiDevice.get_type_function();
    config_ctx.wifi_connection_status = EiDevice.get_wifi_connection_status_function(); 
    config_ctx.wifi_present = EiDevice.get_wifi_present_status_function();
    config_ctx.load_config = &ei_zephyr_flash_load_config;
    config_ctx.save_config = &ei_zephyr_flash_save_config;
    config_ctx.list_files = list_files_help_fn;
    config_ctx.read_buffer = EiDevice.get_read_sample_buffer_function();


    EI_CONFIG_ERROR cr = ei_config_init(&config_ctx);

    if (cr != EI_CONFIG_OK) {
        ei_printf("Failed to initialize configuration (%d)\n", cr);
    }
    else {
        ei_printf("Loaded configuration\n");
    }

    /* Setup the command line commands */
    ei_at_register_generic_cmds();
    ei_at_cmd_register("RUNIMPULSE", "Run the impulse", run_nn_normal);
    //ei_at_cmd_register("RUNIMPULSEDEBUG", "Run the impulse with extra debug output", run_nn_debug);
    ei_at_cmd_register("RUNIMPULSECONT", "Run the impulse continuously", run_nn_continuous_normal);
    ei_printf("Type AT+HELP to see a list of commands.\r\n> ");

    EiDevice.set_state(eiStateFinished);
}

void ei_main(void)
{
    /* handle command comming from uart */
    ei_command_line_handle();
    /* handle command comming from BLE NUS */
    if(ei_ble_rcv_cmd_flag){
        ei_ble_rcv_cmd_flag = false;
        ei_at_cmd_handle(ei_ble_rcv_cmd_buffer);
        memset(ei_ble_rcv_cmd_buffer, 0x00, sizeof(ei_ble_rcv_cmd_buffer));
    }

}
