/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#ifndef SPI_NAND_FLASH_H_
#define SPI_NAND_FLASH_H_

/**@file
 * @defgroup spi_nand_flash SPI NAND Flash Driver API
 * @{
 * @brief API for the SPI NAND Flash driver.
 */

// Include necessary headers
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define EIGHTKFREQUENCY 8000000

    static struct spi_config spi_cfg = {
        .frequency = 8000000,                            // Set the SPI bus frequency to 8 MHz
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB, // Set SPI operation mode: 8-bit words, MSB first
        .slave = 0,                                      // Specify the slave device ID
    };
#define SPI0_NODE DT_NODELABEL(spi3)
    static const struct device *spi0_dev = DEVICE_DT_GET(SPI0_NODE);
#define GPIO0_NODE DT_NODELABEL(gpio0)
#define GPIO0_CS 17

    static const struct device *gpio0_dev = DEVICE_DT_GET(GPIO0_NODE);

#define SLEEP_TIME_MS 1000

#define FLASH_WP_PIN NRF_GPIO_PIN_MAP(0, 22) // IO_2

#define FLASH_HOLD_PIN NRF_GPIO_PIN_MAP(0, 23) // IO_3
#define FLASH_TRIGGER NRF_GPIO_PIN_MAP(0, 25)  // flash trigger

#define BAD_BLOCK_MARK 0x00

// status value
#define ECC_STATUS_NO_ERR 0b000
#define ECC_STATUS_1_3_NO_REFRESH 0b001
#define ECC_STATUS_4_6_REFRESH 0b011
#define ECC_STATUS_7_8_REFRESH 0b101
#define ECC_STATUS_NOT_CORRECTED 0b010

    /* Manufacture ID and Device ID */
    typedef enum
    {
        MFR_ID_MICRON = 0x2C,
        DEVICE_ID_1G_1V8 = 0x35,

    } mfrId_deviceId;

#define SPI_NAND_OOB_SIZE 256
#define SPI_NAND_PAGE_SIZE 4096
#define SPI_NAND_PAGES_PER_BLOCK 64
#define SPI_NAND_BLOCKS_PER_LUN 2048
#define SPI_NAND_MAX_PAGE_ADDRESS (SPI_NAND_PAGES_PER_BLOCK - 1) /* zero-indexed */
#define SPI_NAND_MAX_BLOCK_ADDRESS (SPI_NAND_BLOCKS_PER_LUN - 1) /* zero-indexed */

#define SPI_NAND_LOG2_PAGE_SIZE 12
#define SPI_NAND_LOG2_PAGES_PER_BLOCK 6

    /**
     *@brief SPI return statuses
     */
    typedef enum
    {
        SPI_NAND_RET_OK = 0,
        SPI_NAND_RET_BAD_SPI = -1,
        SPI_NAND_RET_TIMEOUT = -2,
        SPI_NAND_RET_DEVICE_ID = -3,
        SPI_NAND_RET_BAD_ADDRESS = -4,
        SPI_NAND_RET_INVALID_LEN = -5,
        SPI_NAND_RET_ECC_REFRESH = -6,
        SPI_NAND_RET_ECC_ERR = -7,
        SPI_NAND_RET_P_FAIL = -8,
        SPI_NAND_RET_E_FAIL = -9,
        RESET_ERROR = -10,
        READ_ID_ERROR = -11,
        UNLOCK_ALL_BLOCK_ERROR = -12,
        ENABLE_ECC_ERROR = -13,
        WRITE_ENABLE_ERROR = -14,
        PROGRAM_LOAD_ERROR = -15,
        PROGRAM_EXECUTE_ERROR = -16,
        READ_PAGE_ERROR = -17,
        READ_FROM_CACHE_ERROR = -18,
        GET_FEATURE_ERROR = -19,
        SET_FEATURE_ERROR = -20,
        OIP_POLL_ERROR = -21,
        BLOCK_ERASE_ERROR = -22,
        FLASH_READ_ERROR = -23,
        PROGRAM_LOAD_RANDOM_DATA_ERROR = -24,
        STATUS_FAIL = -25,
    } ReturnType;

#define OP_TIMEOUT 3000

    typedef enum
    {
        READ_ID_TRANS_LEN = 4,
        READ_ID_MFR_INDEX = 2,
        READ_ID_DEVICE_INDEX = 3,
        FEATURE_TRANS_LEN = 3,
        FEATURE_REG_INDEX = 1,
        FEATURE_DATA_INDEX = 2,
        PAGE_READ_TRANS_LEN = 4,
        READ_FROM_CACHE_TRANS_LEN = 4,
        PROGRAM_LOAD_TRANS_LEN = 3,
        PROGRAM_EXECUTE_TRANS_LEN = 4,
        BLOCK_ERASE_TRANS_LEN = 4,
        PROGRAM_LOAD_RANDOM_DATA_TRANS_LEN = 3,

    } length_index;

    typedef enum
    {
        FEATURE_REG_BLOCK_LOCK = 0xA0,
        FEATURE_REG_CONFIGURATION = 0xB0,
        FEATURE_REG_STATUS = 0xC0,
    } Register;

    // Commands
    // cmd
    enum
    {
        CMD_RESET = 0xFF,                   /*RESET COMMAND*/
        CMD_READ_ID = 0x9F,                 /*READ ID COMMAND*/
        CMD_SET_FEATURE = 0x1F,             /*SET FEATURE COMMAND*/
        CMD_GET_FEATURE = 0x0F,             /*GET FEATURE COMMAND*/
        CMD_PAGE_READ = 0x13,               /*READ PAGE COMMAND*/
        CMD_READ_FROM_CACHE = 0x03,         /*READ FROM CACHE COMMAND*/
        CMD_WRITE_ENABLE = 0x06,            /*WRITE ENABLE COMMAND*/
        CMD_PROGRAM_LOAD = 0x02,            /*LOAD PROGRAM COMMAND*/
        CMD_PROGRAM_EXECUTE = 0x10,         /*RUN PROGRAM COMMAND*/
        CMD_BLOCK_ERASE = 0xD8,             /*ERASE BLOCK COMMAND*/
        CMD_PROGRAM_LOAD_RANDOM_DATA = 0x84 /*LOAD RANDOM DATA COMMAND*/
    };

    /** @brief Structure representing the configuration feature register. */
    typedef union
    {
        uint8_t whole;
        struct
        {
            uint8_t : 1;
            uint8_t CFG0 : 1;
            uint8_t : 2;
            uint8_t ECC_EN : 1;
            uint8_t LOT_EN : 1;
            uint8_t CFG1 : 1;
            uint8_t CFG2 : 1;
        };
    } feature_reg_configuration_t;

    /** @brief Structure representing a row address in NAND flash. */
    typedef union
    {
        uint32_t whole;
        struct
        {
            uint32_t page : 6;   /* Valid range 0-63  */
            uint32_t block : 26; /* Valid range 0-2047 */
        };
    } row_address_t;

    /** @brief Type representing a column address in NAND flash. */
    typedef uint16_t column_address_t;

    /** @brief Structure representing the block lock feature register. */
    typedef union
    {
        uint8_t whole;
        struct
        {
            uint8_t : 1;
            uint8_t WP_HOLD_DISABLE : 1;
            uint8_t TB : 1;
            uint8_t BP0 : 1;
            uint8_t BP1 : 1;
            uint8_t BP2 : 1;
            uint8_t BP3 : 1;
            uint8_t BRWD : 1;
        };
    } feature_reg_block_lock_t;

    /** @brief Structure representing the status feature register. */
    typedef union
    {
        uint8_t whole;
        struct
        {
            uint8_t OIP : 1;
            uint8_t WEL : 1;
            uint8_t E_FAIL : 1;
            uint8_t P_FAIL : 1;
            uint8_t ECCS0_3 : 3;
            uint8_t CRBSY : 1;
        };
    } feature_reg_status_t;

#define CS_SELECT(pin)                   \
    do                                   \
    {                                    \
        gpio_pin_set(gpio0_dev, pin, 0); \
    } while (0)

#define CS_DESELECT(pin)                 \
    do                                   \
    {                                    \
        gpio_pin_set(gpio0_dev, pin, 1); \
    } while (0)

    ReturnType MT29F4G_flash_init(void);
    ReturnType MT29F4G_flash_write_page(row_address_t row, column_address_t column, uint8_t *data_in, size_t write_len);
    ReturnType MT29F4G_flash_page_read(row_address_t row, column_address_t column, uint8_t *data_out, size_t read_len);
    ReturnType MT29F4G_flash_block_erase(row_address_t row);
    ReturnType MT29F4G_flash_is_bad_block(row_address_t row, bool *is_bad);
    ReturnType MT29F4G_flash_flashblock_mark_bad(row_address_t row);
    ReturnType MT29F4G_flash_flashpage_is_free(row_address_t row, bool *is_free);
    ReturnType MT29F4G_flash_page_copy(row_address_t src, row_address_t dest);
    ReturnType MT29F4G_flash_chip_erase(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* SPI_NAND_FLASH_H_ */