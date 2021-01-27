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

/* Include ----------------------------------------------------------------- */
#include "ei_device_nordic_nrf52.h"
#include "ei_zephyr_flash_commands.h"
//#include "retargetserial.h"
#include "ei_microphone.h"
#include "ei_inertialsensor.h"
#include "repl.h"

#include <cstdarg>
#include "math.h"

#include <sys/printk.h>
#include <drivers/uart.h>

extern "C" void ei_led_state_control(void);

/* Constants --------------------------------------------------------------- */
#define EI_LED_BLUE     BOARD_ledSetLedOn(1, 0, 0, 0)
#define EI_LED_GREEN    BOARD_ledSetLedOn(0, 1, 0, 0)
#define EI_LED_YELLOW   BOARD_ledSetLedOn(0, 0, 1, 0)
#define EI_LED_RED      BOARD_ledSetLedOn(0, 0, 0, 1)
#define EI_LED_OFF      BOARD_ledSetLedOn(0, 0, 0, 0)

/** Led device struct */
const struct device *led_dev;
const struct device *uart;

/** Max size for device id array */
#define DEVICE_ID_MAX_SIZE  32

/** Sensors */
typedef enum
{
    ACCELEROMETER = 0,
    MICROPHONE

}used_sensors_t;

#define EDGE_STRINGIZE_(x) #x
#define EDGE_STRINGIZE(x) EDGE_STRINGIZE_(x)

/** Device type */
#if ((CONFIG_SOC_NRF52840 == 1) || \
       (CONFIG_SOC_NRF52840_QIAA == 1))
static const char *ei_device_type = "NRF52840_DK  ";
#elif ((CONFIG_SOC_NRF5340_CPUAPP == 1) || \
       (CONFIG_SOC_NRF5340_CPUAPP_QKAA == 1))
static const char *ei_device_type = "NRF5340_DK   ";
#else 
#error "Unsupported build target was chosen!"
#endif


/** Device id array */
static char ei_device_id[DEVICE_ID_MAX_SIZE];

/** Device object, for this class only 1 object should exist */
EiDeviceNRF52 EiDevice;

static tEiState ei_program_state = eiStateIdle;

/* Private function declarations ------------------------------------------- */
static int get_id_c(uint8_t out_buffer[32], size_t *out_size);
static int get_type_c(uint8_t out_buffer[32], size_t *out_size);
static bool get_wifi_connection_status_c(void);
static bool get_wifi_present_status_c(void);
static bool read_sample_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t));
static void zephyr_timer_handler(struct k_timer *dummy);

/** Zephyr timer */
K_TIMER_DEFINE(led_timer, zephyr_timer_handler, NULL);

/* Public functions -------------------------------------------------------- */

EiDeviceNRF52::EiDeviceNRF52(void)
{
    /* Read nordic id from the registers */
#if ((CONFIG_SOC_NRF52840 == 1) || \
       (CONFIG_SOC_NRF52840_QIAA == 1))
    uint32_t id_msb = (uint32_t)NRF_FICR->DEVICEID[1];
    uint32_t id_lsb = (uint32_t)NRF_FICR->DEVICEID[0];
#elif ((CONFIG_SOC_NRF5340_CPUAPP == 1) || \
       (CONFIG_SOC_NRF5340_CPUAPP_QKAA == 1))
    uint32_t id_msb = (uint32_t)NRF_FICR->INFO.DEVICEID[1];
    uint32_t id_lsb = (uint32_t)NRF_FICR->INFO.DEVICEID[0];
#else 
#error "Unsupported build target was chosen!"
#endif

    /* Setup device ID */
    snprintf(&ei_device_id[0], DEVICE_ID_MAX_SIZE, 
             "%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned int)((id_msb >> 8) & 0xFF),
             (unsigned int)((id_msb >> 0) & 0xFF),
             (unsigned int)((id_lsb >> 24)& 0xFF),
             (unsigned int)((id_lsb >> 16)& 0xFF),
             (unsigned int)((id_lsb >> 8) & 0xFF),
             (unsigned int)((id_lsb >> 0) & 0xFF));
}

/**
 * @brief      For the device ID, the BLE mac address is used.
 *             The mac address string is copied to the out_buffer.
 *
 * @param      out_buffer  Destination array for id string
 * @param      out_size    Length of id string
 *
 * @return     0
 */
int EiDeviceNRF52::get_id(uint8_t out_buffer[32], size_t *out_size)
{
    return get_id_c(out_buffer, out_size);
}

/**
 * @brief      Gets the identifier pointer.
 *
 * @return     The identifier pointer.
 */
const char *EiDeviceNRF52::get_id_pointer(void)
{
    return (const char *)ei_device_id;
}

/**
 * @brief      Copy device type in out_buffer & update out_size
 *
 * @param      out_buffer  Destination array for device type string
 * @param      out_size    Length of string
 *
 * @return     -1 if device type string exceeds out_buffer
 */
int EiDeviceNRF52::get_type(uint8_t out_buffer[32], size_t *out_size)
{
    return get_type_c(out_buffer, out_size);
}

/**
 * @brief      Gets the type pointer.
 *
 * @return     The type pointer.
 */
const char *EiDeviceNRF52::get_type_pointer(void)
{
    return (const char *)ei_device_type;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceNRF52::get_wifi_connection_status(void)
{
    return false;
}

/**
 * @brief      No Wifi available for device.
 *
 * @return     Always return false
 */
bool EiDeviceNRF52::get_wifi_present_status(void)
{
    return false;
}

/**
 * @brief      Create sensor list with sensor specs
 *             The studio and daemon require this list
 * @param      sensor_list       Place pointer to sensor list
 * @param      sensor_list_size  Write number of sensors here
 *
 * @return     False if all went ok
 */
bool EiDeviceNRF52::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    /* Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead */
    uint32_t available_bytes = (ei_zephyr_flash_get_n_available_sample_blocks()-1) * ei_zephyr_flash_get_block_size();

    sensors[ACCELEROMETER].name = "Accelerometer";
    sensors[ACCELEROMETER].start_sampling_cb = &ei_inertial_setup_data_sampling;
    sensors[ACCELEROMETER].max_sample_length_s = available_bytes / (100 * SIZEOF_N_AXIS_SAMPLED);
    sensors[ACCELEROMETER].frequencies[0] = 62.5f;
    sensors[ACCELEROMETER].frequencies[1] = 100.0f;

    sensors[MICROPHONE].name = "Microphone";
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_sample_start;
    sensors[MICROPHONE].max_sample_length_s = available_bytes / (16000 * 2);
    sensors[MICROPHONE].frequencies[0] = 16000.0f;

    *sensor_list      = sensors;
    *sensor_list_size = EI_DEVICE_N_SENSORS;

    return false;
}

/**
 * @brief      Device specific delay ms implementation
 *
 * @param[in]  milliseconds  The milliseconds
 */
void EiDeviceNRF52::delay_ms(uint32_t milliseconds)
{
    k_msleep((int32_t)milliseconds);
}

/**
 * @brief      Device specific get ms form application start
 *
 * @return    The milliseconds form start
 */
uint64_t EiDeviceNRF52::get_ms(void)
{
    return (uint64_t)k_uptime_get();
}

void EiDeviceNRF52::set_state(tEiState state)
{
    ei_program_state = state;

    if(state == eiStateFinished) {
        EI_LED_OFF;
        delay_ms(300);
        EI_LED_BLUE;
        delay_ms(300);
        EI_LED_GREEN;
        delay_ms(300);
        EI_LED_YELLOW;
        delay_ms(300);
        EI_LED_RED;
        delay_ms(300);
        EI_LED_OFF;

        ei_program_state = eiStateIdle;
    }
}

/**
 * @brief      Get a C callback for the get_id method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceNRF52::get_id_function(void)
{
    return &get_id_c;
}

/**
 * @brief      Get a C callback for the get_type method
 *
 * @return     Pointer to c get function
 */
c_callback EiDeviceNRF52::get_type_function(void)
{
    return &get_type_c;
}

/**
 * @brief      Get a C callback for the get_wifi_connection_status method
 *
 * @return     Pointer to c get function
 */
c_callback_status EiDeviceNRF52::get_wifi_connection_status_function(void)
{
    return &get_wifi_connection_status_c;
}

/**
 * @brief      Get a C callback for the wifi present method
 *
 * @return     The wifi present status function.
 */
c_callback_status EiDeviceNRF52::get_wifi_present_status_function(void)
{
    return &get_wifi_present_status_c;
}

/**
 * @brief      Get a C callback to the read sample buffer function
 *
 * @return     The read sample buffer function.
 */
c_callback_read_sample_buffer EiDeviceNRF52::get_read_sample_buffer_function(void)
{
    return &read_sample_buffer;
}

/**
 * @brief      Write serial data with length to Serial output
 *
 * @param      data    The data
 * @param[in]  length  The length
 */
void ei_write_string(char *data, int length) 
{
    for( int i = 0; i < length; i++) {
        printf("%c", *(data++));
    }
}


void ei_printfloat(int n_decimals, int n, ...)
{
    int i;
    double val;

    char buffer[32];

    sprintf(buffer, "%%.%df", n_decimals);

    va_list vl;
    va_start(vl,n);
    for (i=0;i<n;i++){
        val=va_arg(vl,double);
        ei_printf(buffer, val);
    }
    va_end(vl);
}

void ei_printf_float(float f)
{
    float n = f;

    static double PRECISION = 0.00001;
    static int MAX_NUMBER_STRING_SIZE = 32;

    char s[MAX_NUMBER_STRING_SIZE];

    if (n == 0.0) {
        ei_printf("0.00000");
    }
    else {
        int digit, m;//, m1;
        char *c = s;
        int neg = (n < 0);
        if (neg) {
            n = -n;
        }
        // calculate magnitude
        m = log10(n);
        if (neg) {
            *(c++) = '-';
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISION || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0) {
                *(c++) = '.';
            }
            m--;
        }
        *(c) = '\0';
        ei_write_string(s, c - s);
    }    
}

/**
 * @brief      Get characters for uart pheripheral and send to repl
 */
void ei_command_line_handle(void)
{
    char data = uart_getchar();

    while(data != 0xFF) {
        rx_callback(data);
        data = uart_getchar();
    }
}

bool ei_user_invoke_stop(void)
{
    bool stop_found = false;
    char data = uart_getchar();
    
    while(data != 0xFF) {
        if(data == 'b') {
            stop_found = true;
            break;
        }
        data = uart_getchar();
    }

    return stop_found;
}

/**
 * @brief      Check if new serial data is available
 *
 * @return     Returns number of available bytes
 */
int ei_get_serial_available(void) {
    return 0;//Serial.available();
}

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void) {
    return 0;//Serial.read();
}

/* Private functions ------------------------------------------------------- */

static int get_id_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_id);

    if(length < 32) {
        memcpy(out_buffer, ei_device_id, length);

        *out_size = length;
        return 0;
    }
    else {
        *out_size = 0;
        return -1;
    }
}

static int get_type_c(uint8_t out_buffer[32], size_t *out_size)
{
    size_t length = strlen(ei_device_type);

    if(length < 32) {
        memcpy(out_buffer, ei_device_type, length);

        *out_size = length;
        return 0;
    }
    else {
        *out_size = 0;
        return -1;
    }
}

static bool get_wifi_connection_status_c(void)
{
    return false;
}

static bool get_wifi_present_status_c(void)
{
    return false;
}

/**
 * @brief      Read samples from sample memory and send to data_fn function
 *
 * @param[in]  begin    Start address
 * @param[in]  length   Length of samples in bytes
 * @param[in]  data_fn  Callback function for sample data
 *
 * @return     false on flash read function
 */
static bool read_sample_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t))
{
    size_t pos = begin;
    size_t bytes_left = length;
    bool retVal;

    EiDevice.set_state(eiStateUploading);

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    while (1) {
        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            retVal = true;
            break;
        }

        int r = ei_zephyr_flash_read_samples(buffer, pos, bytes_to_read);
        if (r != 0) {
            retVal = false;
            break;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    EiDevice.set_state(eiStateFinished);

    return retVal;
}

void ei_led_state_control(void)
{
    static char toggle = 0;


    if(toggle) {
        switch(ei_program_state)
        {
            case eiStateErasingFlash:   EI_LED_BLUE;    break;
            case eiStateSampling:       EI_LED_YELLOW;  break;
            case eiStateUploading:      EI_LED_RED;     break;
            default: break;
        }
    }
    else {
        if(ei_program_state != eiStateFinished) {
            EI_LED_OFF;
        }
    }
    toggle ^= 1;
}

/**
 * @brief      Peridioc (200ms) handler for the LED's
 *
 * @param      dummy  The dummy
 */
static void zephyr_timer_handler(struct k_timer *dummy)
{
    ei_led_state_control();
}

/**
 * @brief      Sets development kit LEDs on and off
 *
 * @param[in]  led1     set LED1 on and off (true/false)
 * @param[in]  led2     set LED2 on and off (true/false)
 * @param[in]  led3     set LED3 on and off (true/false)
 * @param[in]  led4     set LED4 on and off (true/false)
 *
 */
void BOARD_ledSetLedOn(uint8_t led1, uint8_t led2, uint8_t led3, uint8_t led4)
{
    /** set led1 */
    if(led1 < 1) gpio_pin_set(led_dev, PIN_LED0, 0); else gpio_pin_set(led_dev, PIN_LED0, 1);
    /** set led3 */
    if(led2 < 1) gpio_pin_set(led_dev, PIN_LED1, 0); else gpio_pin_set(led_dev, PIN_LED1, 1);
    /** set led3 */
    if(led3 < 1) gpio_pin_set(led_dev, PIN_LED2, 0); else gpio_pin_set(led_dev, PIN_LED2, 1);
    /** set led4 */
    if(led4 < 1) gpio_pin_set(led_dev, PIN_LED3, 0); else gpio_pin_set(led_dev, PIN_LED3, 1);
}

/**
 * @brief      Init development kit LEDs
 *
 * @return     0 If successful
 * @return     EIO If not successful
 *
 */
int BOARD_ledInit(void)
{
    int ret = 0;

    led_dev = device_get_binding(LED_DEVICE);
    if(led_dev == NULL)
    {
        return EIO;
    }
    /** init gpio for led0 */
    ret = gpio_pin_configure(led_dev, PIN_LED0, GPIO_OUTPUT_ACTIVE | FLAGS_LED0);
    if(ret < 0)
    {
        return EIO;
    }
    /** init gpio for led1 */
    ret = gpio_pin_configure(led_dev, PIN_LED1, GPIO_OUTPUT_ACTIVE | FLAGS_LED1);
    if(ret < 0)
    {
        return EIO;
    }
    /** init gpio for led2 */
    ret = gpio_pin_configure(led_dev, PIN_LED2, GPIO_OUTPUT_ACTIVE | FLAGS_LED2);
    if(ret < 0)
    {
        return EIO;
    }
    /** init gpio for led3 */
    ret = gpio_pin_configure(led_dev, PIN_LED3, GPIO_OUTPUT_ACTIVE | FLAGS_LED3);
    if(ret < 0)
    {
        return EIO;
    }

    /* start periodic timer that expires once every second */
    k_timer_start(&led_timer, K_MSEC(200), K_MSEC(200));

    return ret;
}

/**
 * @brief      Init development kit UART
 *
 * @return     0 If successful
 * @return     -ENXIO If not successful
 *
 */
int uart_init(void)
{
	int err = 0;

	uart = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));
	if (!uart) {
		return -ENXIO;
	}
    
    return err;
}

/**
 * @brief      Get char from UART
 *
 * @return     rcv_char If successful
 * @return     0xFF If not successful
 *
 */
char uart_getchar(void)
{
    unsigned char rcv_char;

    if (!uart_poll_in(uart, &rcv_char)) {
        return rcv_char;
    }
    else{
        return 0xFF;
    }
}

/**
 * @brief      Get char from UART
 *
 * @param[in] send_char Character to be sent over UART
 *
 */
void uart_putchar(char send_char)
{
    uart_poll_out(uart, send_char);
}
