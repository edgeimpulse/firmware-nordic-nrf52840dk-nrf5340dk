
/* Include ----------------------------------------------------------------- */
#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <device.h>

#include "ei_zephyr_flash_commands.h"
#include "ei_device_nordic_nrf52.h"

#define SERIAL_FLASH            0
#define MICRO_SD                1
#define RAM                     2

#define RAM_SAMPLE_BLOK         20
#define RAM_SAMPLE_BLOK_SIZE    4096

#define SAMPLE_MEMORY           SERIAL_FLASH
#define SIZE_RAM_BUFFER         (RAM_SAMPLE_BLOK_SIZE * RAM_SAMPLE_BLOK)


/* Private function prototypes --------------------------------------------- */

#if(SAMPLE_MEMORY == SERIAL_FLASH)
static int flash_erase_sectors(uint32_t address_offset, uint32_t num_sectors);
static int flash_read_data(uint8_t *buffer,
                           uint32_t address_offset,
                           uint32_t num_bytes);
static int flash_write_data(uint8_t *buffer,
                            uint32_t address_offset,
                            uint32_t num_bytes);
#endif

#if(SAMPLE_MEMORY == RAM)
static uint8_t ram_memory[SIZE_RAM_BUFFER];
#endif

/** 32-bit align write buffer size */
#define WORD_ALIGN(a)	((a & 0x3) ? (a & ~0x3) + 0x4 : a)

const static struct device *flash_dev;

/*!
 * @brief       Gets binding for flash device and stores it to static
 *              device variable
 */
void create_flash_device()
{
#if SAMPLE_MEMORY == SERIAL_FLASH
    flash_dev = device_get_binding(EXTERNAL_FLASH_DEVICE);
    ei_printf("Using flash device: " EXTERNAL_FLASH_DEVICE "\n");
    //ei_printf("Flash device size: %d bytes\n", ZEPHYR_TOTAL_FLASH_SIZE);
    //ei_printf("Flash device sector size: %d bytes\n", EXTERNAL_FLASH_DEVICE_SECTOR_SIZE);
#endif
}


/**
 * @brief      Copy configuration data to config pointer
 *
 * @param      config       Destination pointer for config
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_zephyr_ret_t enum
 */
int ei_zephyr_flash_load_config(uint32_t *config, uint32_t config_size)
{
    if(config == NULL) {
        return ZEPHYR_FLASH_CMD_NULL_POINTER;
    }

    #if(SAMPLE_MEMORY == RAM)
    return ZEPHYR_FLASH_CMD_OK;

    #elif(SAMPLE_MEMORY == SERIAL_FLASH)

    return flash_read_data((uint8_t *)config, 0, config_size);
    #endif
}


/**
 * @brief      Write config to Flash
 *
 * @param[in]  config       Pointer to configuration data
 * @param[in]  config_size  Size of configuration in bytes
 *
 * @return     ei_zephyr_ret_t enum
 */
int ei_zephyr_flash_save_config(const uint32_t *config, uint32_t config_size)
{
    int ret_val = ZEPHYR_FLASH_CMD_OK;

    if(config == NULL) {
        return ZEPHYR_FLASH_CMD_NULL_POINTER;
    }

    #if(SAMPLE_MEMORY == RAM)
    return ret_val;
    #elif(SAMPLE_MEMORY == SERIAL_FLASH)
    ret_val = flash_erase_sectors(0, 1);

    if (ret_val == ZEPHYR_FLASH_CMD_OK) {
        return flash_write_data((uint8_t *)config, 0, config_size);
    }
    else {
        return ret_val;
    }
    #endif
}


/**
 * @brief      Erase blocks in sample data space
 *
 * @param[in]  start_block  The start block
 * @param[in]  end_address  The end address
 *
 * @return     ei_zephyr_ret_t
 */
int ei_zephyr_flash_erase_sampledata(uint32_t start_block,
                                     uint32_t end_address)
{
    #if(SAMPLE_MEMORY == RAM)
    return ZEPHYR_FLASH_CMD_OK;
    #elif(SAMPLE_MEMORY == SERIAL_FLASH)
    return flash_erase_sectors(EI_DATA_SAMPLES_OFFSET,
                               end_address / ei_zephyr_flash_get_block_size());
    #endif
}


/**
 * @brief      Write sample data to flash
 *
 * @param[in]  buffer          Buffer holding data that we want to write
 * @param[in]  address_offset  The offset that we want to write from
 * @param[in]  num_write_bytes The number of bytes that we want to write
 *
 * @return     ei_zephyr_ret_t
 */
int ei_zephyr_flash_write_samples(const void *sample_buffer,
                                  uint32_t address_offset,
                                  uint32_t num_write_bytes)
{
    uint32_t aligned_num_write_bytes = WORD_ALIGN(num_write_bytes);
    //Debug
    // ei_printf("ei_zephyr_flash_write_samples\n");
    // ei_printf("Input arguments:\n");
    // ei_printf("address_offest is 0x%X\n", address_offset);
    // ei_printf("aligned_num_write_bytes: %d\n", aligned_num_write_bytes);

    #if(SAMPLE_MEMORY == RAM)
    if((address_offset + aligned_num_write_bytes) > SIZE_RAM_BUFFER) {
        return ZEPHYR_FLASH_CMD_WRITE_ERROR;
    }
    else if(sample_buffer == 0) {
        return ZEPHYR_FLASH_CMD_NULL_POINTER;
    }

    for(uint32_t i = 0;  i < aligned_num_write_bytes; i++) {
        ram_memory[address_offset + i] = *((char *)sample_buffer + i);
    }
    return ZEPHYR_FLASH_CMD_OK;

    #elif(SAMPLE_MEMORY == SERIAL_FLASH)
    //ei_printf("%s: 0x%x 0x%x 0x%x 0x%x\n", __FUNCTION__, *((char *)sample_buffer + 0), *((char *)sample_buffer + 1), *((char *)sample_buffer + 2), *((char *)sample_buffer + 3));
    return flash_write_data((uint8_t *)sample_buffer,
                            EI_DATA_SAMPLES_OFFSET + address_offset,
                            aligned_num_write_bytes);
    #endif
}


/**
 * @brief      Read sample data from flash
 *
 * @param      sample_buffer   The sample buffer
 * @param[in]  address_offset  The address offset
 * @param[in]  n_read_bytes    The n read bytes
 *
 * @return     ei_zephyr_ret_t
 */
int ei_zephyr_flash_read_samples(void *sample_buffer,
                                 uint32_t address_offset,
                                 uint32_t num_read_bytes)
{
    uint32_t aligned_num_read_bytes = WORD_ALIGN(num_read_bytes);
    //ei_printf("ei_zephyr_flash_read_sample_data\n");
    //ei_printf("Input arguments:\n");
    //ei_printf("address_offest is 0x%X\n", address_offset);
    //ei_printf("n_read_bytes  is %d\n\n", num_read_bytes);

    #if(SAMPLE_MEMORY == RAM)
    if((address_offset + num_read_bytes) > SIZE_RAM_BUFFER) {
        return ZEPHYR_FLASH_CMD_READ_ERROR;
    }
    else if(sample_buffer == 0) {
        return ZEPHYR_FLASH_CMD_NULL_POINTER;
    }

    for(uint32_t i = 0;  i < aligned_num_read_bytes; i++) {
        *((char *)sample_buffer + i) = ram_memory[address_offset + i];
    }
    return ZEPHYR_FLASH_CMD_OK;

    #elif(SAMPLE_MEMORY == SERIAL_FLASH)

    int rc = flash_read_data((uint8_t *)sample_buffer,
                           EI_DATA_SAMPLES_OFFSET + address_offset,
                           aligned_num_read_bytes);

    return rc;
    #endif
}

/**
 * @brief      Get block size (Smallest erasble block).
 *
 * @return     Length of 1 block
 */
uint32_t ei_zephyr_flash_get_block_size(void)
{
    #if(SAMPLE_MEMORY == RAM)
    return RAM_SAMPLE_BLOK_SIZE;
    #elif(SAMPLE_MEMORY == SERIAL_FLASH)
    return ZEPHYR_FLASH_SECTOR_SIZE;
    #endif
}

/**
 * @brief      Get available sample blocks
 *
 * @return     Sample memory size / block size
 */
uint32_t ei_zephyr_flash_get_n_available_sample_blocks(void)
{
    #if(SAMPLE_MEMORY == RAM)
    return RAM_SAMPLE_BLOK;
    #elif(SAMPLE_MEMORY == SERIAL_FLASH)    
    return (ZEPHYR_TOTAL_FLASH_SIZE - EI_DATA_SAMPLES_OFFSET) /
            ZEPHYR_FLASH_SECTOR_SIZE;
    #endif
}



//#define FLASH_TEST_OFFSET FLASH_AREA_OFFSET(image_1)
#define FLASH_TEST_OFFSET 0
#define FLASH_PAGE_SIZE   4096
#define TEST_DATA_WORD_0  0x11
#define TEST_DATA_WORD_1  0xaa
#define TEST_DATA_WORD_2  0xab
#define TEST_DATA_WORD_3  0x12

#define FLASH_TEST_OFFSET2 0x41234
#define FLASH_TEST_PAGE_IDX 37

void zephyr_flash_write()
{
}

void zephyr_flash_write1(uint8_t * buf)
{
    uint32_t offset;
    uint8_t temp = 0x42;
    // uint32_t buf_array_1[4] = { TEST_DATA_WORD_0, TEST_DATA_WORD_1,
    //                 TEST_DATA_WORD_2, TEST_DATA_WORD_3 };

    flash_write_protection_set(flash_dev, false);
    for (int i = 0; i < 1; i++) {
        offset = + i;
        ei_printf("   Attempted to write 0x%x at 0x%x\n", temp, offset);
        if (flash_write(flash_dev, offset, &temp, 1)) {
            ei_printf("   Flash write failed!\n");
        }
    }
}

void zephyr_flash_read()
{
    int rc;
    ei_printf("\nTest 1: Flash erase\n");
    flash_write_protection_set(flash_dev, false);

    rc =  ei_zephyr_flash_erase_sampledata(0, 4096);
    //rc = flash_erase_sectors(EI_DATA_SAMPLES_OFFSET, 1);
    if (rc != 0) {
        ei_printf("Flash erase failed! %d\n", rc);
    } else {
        ei_printf("Flash erase succeeded!\n");
    }

    ei_printf("\nTest 3: Writing and reading\n");
    const uint8_t expected[] = {0,1,2,3,4,5,6,7,8, 42, 59, 75, 87};
    const size_t len = sizeof(expected);
    uint8_t buf[sizeof(expected)];
    memset(buf, 0, len);

    ei_printf("Attempting to write %u bytes\n", len);
    ei_zephyr_flash_write_samples(expected, 0, len);
    ei_zephyr_flash_read_samples(buf, 0, len);

    if (memcmp(expected, buf, len) == 0) {
        ei_printf("Data read matches data written. Good!!\n");
        for(int i = 0; i < (int)len; i++) {
            ei_printf("%d\n", buf[i]);
        }
    } else {
        const uint8_t *wp = expected;
        const uint8_t *rp = buf;
        const uint8_t *rpe = rp + len;

        ei_printf("Data read does not match data written!!\n");
        while (rp < rpe) {
            ei_printf("%08x wrote %02x read %02x %s\n",
                   (uint32_t)(0 + (rp - buf)),
                   *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
            ++rp;
            ++wp;
        }
    }
}


#if (SAMPLE_MEMORY == SERIAL_FLASH)
/**
 * @brief      Erase multiple flash sectors
 *
 * @param[in]  startAddress  The start address
 * @param[in]  nSectors      The sectors
 *
 * @return     ei_zephyr_ret_t
 */
static int flash_erase_sectors(uint32_t startAddress, uint32_t nSectors)
{
    flash_write_protection_set(flash_dev, false);

    if (flash_erase(flash_dev,
                    startAddress,
                    ZEPHYR_FLASH_SECTOR_SIZE * nSectors) != 0) {
        flash_write_protection_set(flash_dev, true);
        return ZEPHYR_FLASH_CMD_ERASE_ERROR;
    }
    else {
        flash_write_protection_set(flash_dev, true);
        return ZEPHYR_FLASH_CMD_OK;
    }
}

/**
 * @brief      Read data from flash
 *
 * @param      buffer           Buffer holding read data
 * @param[in]  address_offset   The address offset
 * @param[in]  num_bytes        Number of bytes that we want to read
 *
 * @return     ei_zephyr_ret_t
 */
static int flash_read_data(uint8_t *buffer,
                           uint32_t address_offset,
                           uint32_t num_bytes)
{
    int rc = flash_read(flash_dev, address_offset, buffer, num_bytes);

    if (rc != 0) {
        return ZEPHYR_FLASH_CMD_READ_ERROR;
    }
    else {
        return ZEPHYR_FLASH_CMD_OK;
    }
}


/**
 * @brief      Write data to flash
 *
 * @param      buffer           Buffer holding data that we want to write
 * @param[in]  address_offset   The address offset
 * @param[in]  num_bytes        Number of bytes that we want to read
 *
 * @return     ei_zephyr_ret_t
 */
static int flash_write_data(uint8_t *buffer,
                            uint32_t address_offset,
                            uint32_t num_bytes)
{
    int err = 0;
    //ei_printf("%s: 0x%x 0x%x 0x%x 0x%x\n", __FUNCTION__, buffer[0], buffer[1], buffer[2], buffer[3]);
    err = flash_write_protection_set(flash_dev, false);
    if (err != 0) {
        return ZEPHYR_FLASH_CMD_READ_ERROR;
    }
    //ei_printf("flash_write_protection_set: err: %d\n", err);

    err = flash_write(flash_dev, address_offset, buffer, num_bytes);

    if (err != 0) {
        return ZEPHYR_FLASH_CMD_READ_ERROR;
    }
    else {
        return ZEPHYR_FLASH_CMD_OK;
    }
}

#endif /* #if (SAMPLE_MEMORY == SERIAL_FLASH) */
