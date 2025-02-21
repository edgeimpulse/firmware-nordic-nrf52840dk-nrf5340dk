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

#ifndef EI_ZEPHYR_FLASH_COMMANDS_H
#define EI_ZEPHYR_FLASH_COMMANDS_H

/* Include ----------------------------------------------------------------- */
#include <stdint.h>

#define ZEPHYR_FLASH_BLOCK_ERASE_TIME_MS    90

/** Eta fs return values */
typedef enum
{
    ZEPHYR_FLASH_CMD_OK = 0,                /**!< All is well                */
    ZEPHYR_FLASH_CMD_NOT_INIT,              /**!< FS is not initialised      */
    ZEPHYR_FLASH_CMD_READ_ERROR,            /**!< Error occured during read  */
    ZEPHYR_FLASH_CMD_WRITE_ERROR,           /**!< Error occured during write */
    ZEPHYR_FLASH_CMD_ERASE_ERROR,           /**!< Erase error occured        */
    ZEPHYR_FLASH_CMD_NULL_POINTER,          /**!< Null pointer parsed        */

}ei_zephyr_ret_t;

#define MX25R_RETRY                         (10000)/**!< Number of retries for SPI Flash */

/* Flash device label, that is fetched from device tree */
#define EXTERNAL_FLASH_DEVICE               DT_INST(0, nordic_qspi_nor)           /*< configured in device tree of the board >*/
#define EXTERNAL_FLASH_DEVICE_SIZE          DT_PROP(DT_INST(0, nordic_qspi_nor), size)      /*< configured in device tree of the board >*/
#define EXTERNAL_FLASH_DEVICE_SECTOR_SIZE   CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE   /*< configured in prj.conf of the board >*/

#define ZEPHYR_FLASH_SECTOR_SIZE            (EXTERNAL_FLASH_DEVICE_SECTOR_SIZE)             /*< page size of external flash >*/
#define EI_DATA_SAMPLES_OFFSET              (EXTERNAL_FLASH_DEVICE_SECTOR_SIZE*16)          /*< Offset for stored samples >*/
#define ZEPHYR_TOTAL_FLASH_SIZE             ((EXTERNAL_FLASH_DEVICE_SIZE)/8)                /*< On board Flash size, size in DT is in bits >*/

/* Prototypes -------------------------------------------------------------- */
int ei_zephyr_flash_load_config(uint32_t *config, uint32_t config_size);
int ei_zephyr_flash_save_config(const uint32_t *config, uint32_t config_size);
int ei_zephyr_flash_erase_sampledata(uint32_t start_block,
                                     uint32_t end_address);
int ei_zephyr_flash_read_samples(void *sample_buffer,
                                     uint32_t address_offset,
                                     uint32_t num_read_bytes);
int ei_zephyr_flash_write_samples(const void *sample_buffer,
                                  uint32_t address_offset,
                                  uint32_t num_samples);
uint32_t ei_zephyr_flash_get_block_size(void);
uint32_t ei_zephyr_flash_get_n_available_sample_blocks(void);

void create_flash_device();
void zephyr_flash_write1(uint8_t * buf);
void zephyr_flash_write();
void zephyr_flash_read();

#endif
