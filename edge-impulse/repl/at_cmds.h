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

#ifndef _EDGE_IMPULSE_AT_COMMANDS_CONFIG_H_
#define _EDGE_IMPULSE_AT_COMMANDS_CONFIG_H_

#include "at_cmd_interface.h"
#include "edge-impulse-sdk/porting/lib/at_base64_lib.h"
#include "ei_config.h"

#include "ei_device_nordic_nrf52.h"
#include "ei_zephyr_flash_commands.h"
#include "ei_inertialsensor.h"

#include <zephyr.h>
#include <power/reboot.h>


#define EDGE_IMPULSE_AT_COMMAND_VERSION        "1.6.0" // not a complete set (missing snapshot related commands)
#define ei_putc(c) uart_putchar(c)

static void at_error_not_implemented() {
    ei_printf("Command not implemented\r\n");
}

static void at_clear_config() {
    ei_printf("Clearing config and restarting system...\n");
    ei_config_clear();
    //NVIC_SystemReset();
}

static void at_device_info() {
    uint8_t id_buffer[32] = { 0 };
    size_t id_size;
    int r = ei_config_get_device_id(id_buffer, &id_size);
    if (r == EI_CONFIG_OK) {
        id_buffer[id_size] = 0;
        ei_printf("ID:         %s\n", id_buffer);
    }
    if (ei_config_get_context()->get_device_type == NULL) {
        return;
    }
    r = ei_config_get_context()->get_device_type(id_buffer, &id_size);
    if (r == EI_CONFIG_OK) {
        id_buffer[id_size] = 0;
        ei_printf("Type:       %s\n", id_buffer);
    }
    ei_printf("AT Version: %s\n", EDGE_IMPULSE_AT_COMMAND_VERSION);

    ei_device_data_output_baudrate_t baudrate;
    r = EiDevice.get_data_output_baudrate(&baudrate);
    if (r == 0) {
        ei_printf("Data Transfer Baudrate: %s\r\n", baudrate.str);
    } else {
        ei_printf("Data Transfer Baudrate: UNKNOWN\r\n");
    }
}

static void at_set_device_id(char *device_id) {
    EI_CONFIG_ERROR r = ei_config_set_device_id(device_id);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist Device ID (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }
}

static void at_get_wifi() {
    char *ssid;
    char *password;
    ei_config_security_t security;
    bool connected, present;

    EI_CONFIG_ERROR r = ei_config_get_wifi(&ssid, &password, &security, &connected, &present);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to retrieve WiFi config (%d)\n", r);
        return;
    }
    ei_printf("SSID:      %s\n", ssid);
    ei_printf("Password:  %s\n", password);
    ei_printf("Security:  %d\n", security);

    uint8_t mac_buffer[32] = { 0 };
    size_t mac_size;
    r = ei_config_get_device_id(mac_buffer, &mac_size);
    if (r == EI_CONFIG_OK) {
        mac_buffer[mac_size] = 0;
        ei_printf("MAC:       %s\n", mac_buffer);
    }
    ei_printf("Connected: %d\n", connected);
    ei_printf("Present:   %d\n", present);
}

static void at_set_wifi(char *ssid, char *password, char *security_s) {
    ei_config_security_t security = (ei_config_security_t)atoi(security_s);

    EI_CONFIG_ERROR r = ei_config_set_wifi(ssid, password, security);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist WiFi config (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }
}

static void at_get_sample_settings() {
    char *label;
    float interval;
    uint32_t length;
    char *hmac_key;

    EI_CONFIG_ERROR r = ei_config_get_sample_settings(&label, &interval, &length, &hmac_key);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to retrieve sample settings (%d)\n", r);
        return;
    }
    ei_printf("Label:     %s\n", label);
    ei_printf("Interval:  ");
    ei_printf_float((float)interval);
    ei_printf("ms.\n");
    ei_printf("Length:    %lu ms.\n", length);
    ei_printf("HMAC key:  %s\n", hmac_key);
}

static void at_set_sample_settings(char *label, char *interval_s, char *length_s) {
    float interval = atof(interval_s);
    uint32_t length = (uint32_t)atoi(length_s);
    EI_CONFIG_ERROR r = ei_config_set_sample_settings(label, interval, length);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist sampling settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }
}

static void at_set_sample_settings_w_hmac(char *label, char *interval_s, char *length_s, char *hmac_key) {
    float interval = atof(interval_s);
    uint32_t length = (uint32_t)atoi(length_s);
    EI_CONFIG_ERROR r = ei_config_set_sample_settings(label, interval, length, hmac_key);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist sampling settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }
}

static void at_get_upload_settings() {
    char *api_key;
    char *host;
    char *path;

    EI_CONFIG_ERROR r = ei_config_get_upload_settings(&api_key, &host, &path);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to retrieve upload settings (%d)\n", r);
        return;
    }
    ei_printf("Api Key:   %s\n", api_key);
    ei_printf("Host:      %s\n", host);
    ei_printf("Path:      %s\n", path);
}

static void at_set_upload_host(char *host) {
    EI_CONFIG_ERROR r = ei_config_set_upload_host_settings(host);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist upload settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }
}

static void at_set_upload_settings(char *api_key, char *url) {
    EI_CONFIG_ERROR r = ei_config_set_upload_path_settings(api_key, url);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist upload settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }
}

static void at_get_mgmt_settings() {
    char *mgmt_url;
    bool is_connected;
    char last_error[128] = { '\0' };

    EI_CONFIG_ERROR r = ei_config_get_mgmt_settings(&mgmt_url, &is_connected, last_error, 128);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to retrieve management settings (%d)\n", r);
        return;
    }
    ei_printf("URL:        %s\n", mgmt_url);
    ei_printf("Connected:  %d\n", is_connected);
    ei_printf("Last error: %s\n", last_error);
}

static void at_set_mgmt_settings(char *mgmt_url) {
    EI_CONFIG_ERROR r = ei_config_set_mgmt_settings(mgmt_url);
    if (r != EI_CONFIG_OK) {
        ei_printf("Failed to persist management settings (%d)\n", r);
    }
    else {
        ei_printf("OK\n");
    }
}

static void at_get_snapshot(void) {

    const ei_device_snapshot_resolutions_t *list;
    size_t list_size;
    const char *color_depth;

    int r = EiDevice.get_snapshot_list((const ei_device_snapshot_resolutions_t **)&list, &list_size, &color_depth);
    if (r) { /* apparently false is OK here?! */
        ei_printf("Has snapshot:    0\n");
        return;
    }

    ei_printf("Has snapshot:         1\n");
    ei_printf("Supports stream:      1\n");
    ei_printf("Color depth:          %s\n", color_depth);
    ei_printf("Resolutions:          [ ");
    for (size_t ix = 0; ix < list_size; ix++) {
        ei_printf("%lux%lu", list[ix].width, list[ix].height);
        if (ix != list_size - 1) {
            ei_printf(", ");
        }
        else {
            ei_printf(" ");
        }
    }
    ei_printf("]\n");
}

static void at_take_snapshot(char *width_s, char *height_s, char *baudrate_s) {

    if (!ei_config_get_context()->take_snapshot) {
        at_error_not_implemented();
        return;
    }

    size_t width = (size_t)atoi(width_s);
    size_t height = (size_t)atoi(height_s);

    bool use_max_baudrate = false;
    if (baudrate_s[0] == 'y') {
       use_max_baudrate = true;
    }

    if (!ei_config_get_context()->take_snapshot(width, height, use_max_baudrate)) {
        ei_printf("ERR: Snapshot failed\n");
        return;
    }
}

static void at_start_snapshot_stream(char *width_s, char *height_s, char *baudrate_s) {

    if (!ei_config_get_context()->start_snapshot_stream) {
        at_error_not_implemented();
        return;
    }

    size_t width = (size_t)atoi(width_s);
    size_t height = (size_t)atoi(height_s);

    bool use_max_baudrate = false;
    if (baudrate_s[0] == 'y') {
       use_max_baudrate = true;
    }

    if (!ei_config_get_context()->start_snapshot_stream(width, height, use_max_baudrate)) {
        ei_printf("ERR: Snapshot Stream failed\n");
        return;
    }
}


static void at_list_sensors() {

    const ei_device_sensor_t *list;
    size_t list_size;

    int r = EiDevice.get_sensor_list((const ei_device_sensor_t **)&list, &list_size);
    if (r != 0) {
        ei_printf("Failed to get sensor list (%d)\n", r);
        return;
    }

    for (size_t ix = 0; ix < list_size; ix++) {
        ei_printf("Name: %s, Max sample length: %hus, Frequencies: [", list[ix].name, list[ix].max_sample_length_s);
        for (size_t fx = 0; fx < EI_MAX_FREQUENCIES; fx++) {
            if (list[ix].frequencies[fx] != 0.0f) {
                if (fx != 0) {
                    ei_printf(", ");
                }
                ei_printf_float(list[ix].frequencies[fx]);
                ei_printf("Hz");
            }
        }
        ei_printf("]\n");
    }
}

static void at_list_config() {
    ei_printf("===== Device info =====\n");
    at_device_info();
    ei_printf("\n");
    ei_printf("===== Sensors ======\n");
    at_list_sensors();
    ei_printf("\n");
    ei_printf("===== Snapshot ======\n");
    at_get_snapshot();
    ei_printf("\n");
    ei_printf("===== WIFI =====\n");
    at_get_wifi();
    ei_printf("\n");
    ei_printf("===== Sampling parameters =====\n");
    at_get_sample_settings();
    ei_printf("\n");
    ei_printf("===== Upload settings =====\n");
    at_get_upload_settings();
    ei_printf("\n");
    ei_printf("===== Remote management =====\n");
    at_get_mgmt_settings();
    ei_printf("\n");
}

static void at_list_files_data(char *name) {
    ei_printf("%s\n", name);
}

static void at_list_files() {

    if (!ei_config_get_context()->list_files) {
        at_error_not_implemented();
        return;
    }

    if(ei_config_get_context()->list_files == NULL){
        ei_printf("AT+NACK\n");
    }
    else {
        ei_config_get_context()->list_files(at_list_files_data);
    }
}

static void at_read_file_data(uint8_t *buffer, size_t size) {
    base64_encode((const char*)buffer, size, uart_putchar);
}

static void at_read_file(char *filename, char *baudrate_s) {

    bool use_max_baudrate = false;
    if (baudrate_s[0] == 'y') {
       use_max_baudrate = true;
    }


    // setup data output baudrate
    if (use_max_baudrate) {

        // sleep a little to let the daemon attach on the new baud rate...
        ei_printf("OK\r\n");

        EiDevice.set_max_data_output_baudrate();
        EiDevice.delay_ms(100);
    }

    bool exists = ei_config_get_context()->read_file(filename, at_read_file_data);

    if (use_max_baudrate) {
        // lower baud rate
        ei_printf("\r\nOK\r\n");

        EiDevice.set_default_data_output_baudrate();

        // give some time to re-attach
        EiDevice.delay_ms(100);
    }

    if (!exists) {
        ei_printf("File '%s' does not exist\n", filename);
    }
    else {
        ei_printf("\n");
    }
}

static void at_read_buffer(char *start_s, char *length_s, char *baudrate_s) {

    if (!ei_config_get_context()->read_buffer) {
        at_error_not_implemented();
        return;
    }

    size_t start = (size_t)atoi(start_s);
    size_t length = (size_t)atoi(length_s);

    bool use_max_baudrate = false;
    if (baudrate_s[0] == 'y') {
       use_max_baudrate = true;
    }

    // setup data output baudrate
    if (use_max_baudrate) {

        // sleep a little to let the daemon attach on the new baud rate...
        ei_printf("OK\r\n");
        EiDevice.delay_ms(100);
        EiDevice.set_max_data_output_baudrate();
        EiDevice.delay_ms(100);
    }

    bool success = ei_config_get_context()->read_buffer(start, length, at_read_file_data);

    if (use_max_baudrate) {
        // lower baud rate
        ei_printf("\r\nOK\r\n");

        EiDevice.delay_ms(100);
        EiDevice.set_default_data_output_baudrate();

        // give some time to re-attach
        EiDevice.delay_ms(100);
    }

    if (!success) {
        ei_printf("Failed to read from buffer\n");
    }
    else {
        ei_printf("\n");
    }
}

static void at_read_raw(char *start_s, char *length_s) {
    size_t start = (size_t)atoi(start_s);
    size_t length = (size_t)atoi(length_s);
    unsigned char buffer[32];

    for(; start < length; start += 32) {
        ei_zephyr_flash_read_samples(buffer, start, 32);

        int n_display_bytes = (length - start) < 32 ? (length - start) : 32;
        for(int i=0; i<n_display_bytes; i++) {
            ei_printf("%02X ", (unsigned char)buffer[i]);
        }
        ei_printf("\b\r\n");
    }
}

static void at_unlink_file(char *filename) {
    // bool success = ei_config_get_context()->unlink_file(filename);
    // if (success) {
        ei_printf("\n");
    // }
    // else {
    //     ei_printf("File '%s' could not be unlinked\n", filename);
    // }
}
/*
static void at_upload_file(char *filename) {
    if (!ei_config_get_context()->upload_file) {
        ei_printf("upload_file pointer not set\n");
        return;
    }

    if (!ei_config_get_context()->wifi_connection_status()) {
        ei_printf("Not connected to WiFi, cannot upload\n");
        return;
    }

    ei_printf("Uploading '%s' to %s%s...\n", filename,
        ei_config_get_config()->upload_host,
        ei_config_get_config()->upload_path);

    FILE *file = fopen(filename, "r+");
    if (!file) {
        ei_printf("Failed to upload file, cannot open '%s'\n", filename);
        return;
    }

    bool valid = ei_config_get_context()->upload_file(file, (const char*)filename);
    if (!valid) {
        ei_printf("Failed to upload file\n");
    }
    else {
        ei_printf("File uploaded\n");
    }

    fclose(file);
}
*/
static void at_scan_wifi_data(const char *ssid, ei_config_security_t security, int8_t rssi) {
    ei_printf("SSID: %s, Security: ", ssid);

    switch (security) {
        case EI_SECURITY_NONE: ei_printf("None"); break;
        case EI_SECURITY_WEP: ei_printf("WEP"); break;
        case EI_SECURITY_WPA: ei_printf("WPA"); break;
        case EI_SECURITY_WPA2: ei_printf("WPA2"); break;
        case EI_SECURITY_WPA_WPA2: ei_printf("WPA_WPA2"); break;
        case EI_SECURITY_PAP: ei_printf("PAP"); break;
        case EI_SECURITY_CHAP: ei_printf("CHAP"); break;
        case EI_SECURITY_EAP_TLS: ei_printf("EAP_TLS"); break;
        case EI_SECURITY_PEAP: ei_printf("PEAP"); break;
        default: ei_printf("Unknown"); break;
    }

    ei_printf(" (%d), RSSI: %d dBm\n", security, rssi);
}

static void at_scan_wifi() {

    if (!ei_config_get_context()->scan_wifi) {
        at_error_not_implemented();
        return;
    }

    if (ei_config_get_context()->scan_wifi == NULL) {
        ei_printf("Device does not have a WiFi interface\n");
        return;
    }
    bool success = ei_config_get_context()->scan_wifi(at_scan_wifi_data);
    if (!success) {
        ei_printf("Failed to scan for WiFi networks\n");
    }
}

static void at_sample_start(char *sensor_name) {

    const ei_device_sensor_t *list;
    size_t list_size;

    int r = EiDevice.get_sensor_list((const ei_device_sensor_t **)&list, &list_size);
    if (r != 0) {
        ei_printf("Failed to get sensor list (%d)\n", r);
        return;
    }

    for (size_t ix = 0; ix < list_size; ix++) {
        if (strcmp(list[ix].name, sensor_name) == 0) {
            bool r = list[ix].start_sampling_cb();
            if (!r) {
                ei_printf("Failed to start sampling\n");
            }
            return;
        }
    }

    ei_printf("Failed to find sensor '%s' in the sensor list\n", sensor_name);
}

static void at_clear_files_data(char *filename) {

    if (!ei_config_get_context()->unlink_file)
        at_error_not_implemented();

    if (ei_config_get_context()->unlink_file(filename)) {
        ei_printf("Unlinked '%s'\n", filename);
    }
    else {
        ei_printf("ERR: Failed to unlink '%s'\n", filename);
    }
}

static void at_clear_fs() {

    if (!ei_config_get_context()->list_files) {
        at_error_not_implemented();
        return;
    }

    ei_printf("Clearing file system...\n");

    ei_config_get_context()->list_files(at_clear_files_data);
}

static void at_reset() {
    sys_reboot(SYS_REBOOT_COLD);
}

static void at_boot_mode()
{
    ei_printf("AT+ACK\r");
}

// AT commands related to configuration
void ei_at_register_generic_cmds() {
    ei_at_cmd_register("HELP", "Lists all commands", &ei_at_cmd_print_info);
    ei_at_cmd_register("CLEARCONFIG", "Clears complete config and resets system", &at_clear_config);
    ei_at_cmd_register("CLEARFILES", "Clears all files from the file system, this does not clear config", &at_clear_fs);
    ei_at_cmd_register("CONFIG?", "Lists complete config", &at_list_config);
    ei_at_cmd_register("DEVICEINFO?", "Lists device information", &at_device_info);
    ei_at_cmd_register("DEVICEID=", "Sets the device ID (DEVICEID)", &at_set_device_id);
    ei_at_cmd_register("SENSORS?", "Lists sensors", &at_list_sensors);
    ei_at_cmd_register("RESET", "Reset the system", &at_reset);
    ei_at_cmd_register("WIFI?", "Lists current WiFi credentials", &at_get_wifi);
    ei_at_cmd_register("WIFI=", "Sets current WiFi credentials (SSID,PASSWORD,SECURITY)", &at_set_wifi);
    ei_at_cmd_register("SCANWIFI", "Scans for WiFi networks", &at_scan_wifi);
    ei_at_cmd_register("SAMPLESETTINGS?", "Lists current sampling settings", &at_get_sample_settings);
    ei_at_cmd_register("SAMPLESETTINGS=", "Sets current sampling settings (LABEL,INTERVAL_MS,LENGTH_MS)", &at_set_sample_settings);
    ei_at_cmd_register("SAMPLESETTINGS=", "Sets current sampling settings (LABEL,INTERVAL_MS,LENGTH_MS,HMAC_KEY)",
        &at_set_sample_settings_w_hmac);
    ei_at_cmd_register("UPLOADSETTINGS?", "Lists current upload settings", &at_get_upload_settings);
    ei_at_cmd_register("UPLOADSETTINGS=", "Sets current upload settings (APIKEY,PATH)", &at_set_upload_settings);
    ei_at_cmd_register("UPLOADHOST=", "Sets upload host (HOST)", &at_set_upload_host);
    ei_at_cmd_register("MGMTSETTINGS?", "Lists current management settings", &at_get_mgmt_settings);
    ei_at_cmd_register("MGMTSETTINGS=", "Sets current management settings (URL)", &at_set_mgmt_settings);
    ei_at_cmd_register("SNAPSHOT?", "Lists snapshot settings", &at_get_snapshot);
    ei_at_cmd_register("SNAPSHOT=", "Take a snapshot (WIDTH,HEIGHT,USEMAXRATE?(y/n))", &at_take_snapshot);
    ei_at_cmd_register("SNAPSHOTSTREAM=", "Take a stream of snapshot stream (WIDTH,HEIGHT,USEMAXRATE?(y/n))", &at_start_snapshot_stream);
    ei_at_cmd_register("LISTFILES", "Lists all files on the device", &at_list_files);
    ei_at_cmd_register("READFILE=", "Read a specific file (as base64) (FILENAME,USEMAXRATE?(y/n))", &at_read_file);
    ei_at_cmd_register("READBUFFER=", "Read from the temporary buffer (as base64) (START,LENGTH,USEMAXRATE?(y/n))", &at_read_buffer);
    ei_at_cmd_register("UNLINKFILE=", "Unlink a specific file", &at_unlink_file);
    ei_at_cmd_register("SAMPLESTART=", "Start sampling", &at_sample_start);
    ei_at_cmd_register("READRAW=", "Read raw from flash (START,LENGTH)", &at_read_raw);
    ei_at_cmd_register("BOOTMODE", "Jump to bootloader", &at_boot_mode);
}

#endif // _EDGE_IMPULSE_AT_COMMANDS_CONFIG_H_
