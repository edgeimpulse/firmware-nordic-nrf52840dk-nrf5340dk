/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
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

#ifndef EI_DEVICE_nordic_nrf52_H
#define EI_DEVICE_nordic_nrf52_H

/* Include ----------------------------------------------------------------- */
#include "ei_device_info.h"
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <cstdio>

#define DEFAULT_BAUD 115200
/* 230400 bps seems to be the highest, the safest and most compatible speed
 * for UART. Tests shown that 460800 is not working on nRF5340DK, while 921600
 * is not working on nRF52840DK.
 * See: https://devzone.nordicsemi.com/f/nordic-q-a/76793/baudrate-on-vcom-on-nrf52840dk */
#define MAX_BAUD 230400

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay) && DT_NODE_HAS_STATUS(LED1_NODE, okay) && \
	DT_NODE_HAS_STATUS(LED2_NODE, okay) && DT_NODE_HAS_STATUS(LED3_NODE, okay) 
#define LED_DEVICE	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN_LED0	DT_GPIO_PIN(LED0_NODE, gpios)
#define PIN_LED1	DT_GPIO_PIN(LED1_NODE, gpios)
#define PIN_LED2	DT_GPIO_PIN(LED2_NODE, gpios)
#define PIN_LED3	DT_GPIO_PIN(LED3_NODE, gpios)
#define FLAGS_LED0	DT_GPIO_FLAGS(LED0_NODE, gpios)
#define FLAGS_LED1	DT_GPIO_FLAGS(LED1_NODE, gpios)
#define FLAGS_LED2	DT_GPIO_FLAGS(LED2_NODE, gpios)
#define FLAGS_LED3	DT_GPIO_FLAGS(LED3_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0/1/2/3 devicetree alias is not defined"
#define LED_DEVICE	""
#define PIN_LED0	0
#define FLAGS_LED0	0
#define PIN_LED1	0
#define FLAGS_LED1	0
#define PIN_LED2	0
#define FLAGS_LED2	0
#define PIN_LED3	0
#define FLAGS_LED4	0
#endif

//#define ei_printf(...) printf(__VA_ARGS__)
extern void ei_printf(const char *format, ...);

/** Number of sensors used */
#define EI_DEVICE_N_SENSORS		2

/** EI ingestion and inferencing state */
typedef enum
{
	eiStateIdle 		= 0,
	eiStateErasingFlash,
	eiStateSampling,
	eiStateUploading,
	eiStateFinished

}tEiState;

/** C Callback types */
typedef int (*c_callback)(uint8_t out_buffer[32], size_t *out_size);
typedef bool (*c_callback_status)(void);
typedef bool (*c_callback_read_sample_buffer)(size_t begin, 
                                              size_t length, 
                                              void(*data_fn)(uint8_t*, size_t));

/**
 * @brief      Class description and implementation of device specific 
 * 			   characteristics
 */	
class EiDeviceNRF52 : public EiDeviceInfo
{
private:
	ei_device_sensor_t sensors[EI_DEVICE_N_SENSORS];
public:	
	EiDeviceNRF52(void);
	
	int get_id(uint8_t out_buffer[32], size_t *out_size);
	const char *get_id_pointer(void);
	int get_type(uint8_t out_buffer[32], size_t *out_size);
	const char *get_type_pointer(void);
	bool get_wifi_connection_status(void);
	bool get_wifi_present_status();
	bool get_sensor_list(const ei_device_sensor_t **sensor_list, 
                         size_t *sensor_list_size);
	void delay_ms(uint32_t milliseconds);
	uint64_t get_ms(void);
	void set_state(tEiState state);
	int get_data_output_baudrate(ei_device_data_output_baudrate_t *baudrate);
	void set_default_data_output_baudrate() override;
	void set_max_data_output_baudrate() override;

	c_callback get_id_function(void);
	c_callback get_type_function(void);
	c_callback_status get_wifi_connection_status_function(void);
	c_callback_status get_wifi_present_status_function(void);
	c_callback_read_sample_buffer get_read_sample_buffer_function(void);
	
};

/* Function prototypes ----------------------------------------------------- */
void ei_command_line_handle(void);
bool ei_user_invoke_stop(void);
void ei_write_string(char *data, int length);
void ei_printfloat(int n_decimals, int n, ...);
void ei_printf_float(float f);
void BOARD_ledSetLedOn(uint8_t led1, uint8_t led2, uint8_t led3, uint8_t led4);
int BOARD_ledInit(void);
int uart_init(void);
char uart_getchar(void);
void uart_putchar(char send_char);

/* Reference to object for external usage ---------------------------------- */
extern EiDeviceNRF52 EiDevice;

#endif
