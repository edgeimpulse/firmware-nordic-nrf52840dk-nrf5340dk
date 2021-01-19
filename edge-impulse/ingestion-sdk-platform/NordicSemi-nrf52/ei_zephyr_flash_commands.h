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
#define EXTERNAL_FLASH_DEVICE               DT_LABEL(DT_INST(0, nordic_qspi_nor))           /*< configured in device tree of the board >*/
#define EXTERNAL_FLASH_DEVICE_SIZE          DT_PROP(DT_INST(0, nordic_qspi_nor), size)      /*< configured in device tree of the board >*/
#define EXTERNAL_FLASH_DEVICE_SECTOR_SIZE   CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE   /*< configured in prj.conf of the board >*/

#define ZEPHYR_FLASH_SECTOR_SIZE            (EXTERNAL_FLASH_DEVICE_SECTOR_SIZE)             /*< page size of external flash >*/
#define EI_DATA_SAMPLES_OFFSET              (EXTERNAL_FLASH_DEVICE_SECTOR_SIZE*16)          /*< Offset for stored samples >*/
#define ZEPHYR_TOTAL_FLASH_SIZE	            EXTERNAL_FLASH_DEVICE_SIZE                      /*< On board Flash size >*/

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
