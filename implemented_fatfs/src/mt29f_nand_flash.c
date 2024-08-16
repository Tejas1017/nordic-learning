/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**@file
 * @defgroup spi_nand_flash SPI NAND Flash Driver
 * @{
 * @brief Implementation of the SPI NAND Flash driver.
 */

// Include necessary headers
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>
#include "mt29f_nand_flash.h"
#include "mt29f_nand_flash_reg.h"

/** @brief Register the module for logging. */
LOG_MODULE_REGISTER(spi_flash, LOG_LEVEL_INF);


uint8_t page_main_and_oob_buffer[SPI_NAND_PAGE_SIZE + SPI_NAND_OOB_SIZE];

/**
 * @brief  Initialize the NAND flash.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType MT29F4G_flash_init(void)
{
    k_msleep(2);

    int ret = reset();
    if (SPI_NAND_RET_OK != ret)
    {
        LOG_ERR("unable to reset\n");
        return RESET_ERROR;
    }
    k_msleep(2);
    ret = whoami();
    if (SPI_NAND_RET_OK != ret)
    {
        LOG_ERR("error in reading id\n");
        return READ_ID_ERROR;
    }
    ret = unlock_all_blocks();
    if (SPI_NAND_RET_OK != ret)
    {
        return UNLOCK_ALL_BLOCK_ERROR;
    }
    ret = enable_ecc();
    if (SPI_NAND_RET_OK != ret)
    {
        return ENABLE_ECC_ERROR;
    }
    return SPI_NAND_RET_OK;
}

/**
 * @brief  Write a page to the NAND flash.
 * @param  row: The row address to write to.
 * @param  column: The column address to write to.
 * @param  data_in: The data to be written.
 * @param  write_len: The length of data to write.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType MT29F4G_flash_write_page(row_address_t row, column_address_t column, uint8_t *data_in, size_t write_len)
{
    /* Input validation */
    uint32_t timeout = 0x0;
    if (!validate_row_address(row) || !validate_column_address(column))
    {
        return SPI_NAND_RET_BAD_ADDRESS;
    }
    uint16_t max_write_len = (SPI_NAND_PAGE_SIZE + SPI_NAND_OOB_SIZE) - column;

    if (write_len > max_write_len)
    {
        return SPI_NAND_RET_INVALID_LEN;
    }

    int ret = write_enable(OP_TIMEOUT);

    if (SPI_NAND_RET_OK != ret)
    {
        LOG_ERR("problem in enabling write\n");
        return WRITE_ENABLE_ERROR;
    }

    ret = program_load(column, data_in, write_len, timeout);

    if (SPI_NAND_RET_OK != ret)
    {
        LOG_ERR("error in program load\n");
        return PROGRAM_LOAD_ERROR;
    }

    ret = program_execute(row, timeout);

    if (SPI_NAND_RET_OK != ret)
    {
        LOG_ERR("error in program execute\n");
        return PROGRAM_EXECUTE_ERROR;
    }

    return SPI_NAND_RET_OK;
}

/**
 * @brief  Read a page from the NAND flash.
 * @param  row: The row address to read from.
 * @param  column: The column address to read from.
 * @param  data_out: The data read from the page.
 * @param  read_len: The length of data to read.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType MT29F4G_flash_page_read(row_address_t row, column_address_t column, uint8_t *data_out, size_t read_len)
{
    uint32_t timeout = 0x00;
    uint16_t max_read_len = (SPI_NAND_PAGE_SIZE + SPI_NAND_OOB_SIZE) - column;
    if (!validate_row_address(row) || !validate_column_address(column))
    {
        return SPI_NAND_RET_BAD_ADDRESS;
    }
    if (read_len > max_read_len)
    {
        return SPI_NAND_RET_INVALID_LEN;
    }
    ReturnType ret = page_read(row, OP_TIMEOUT);
    if (SPI_NAND_RET_OK != ret)
    {
        return READ_PAGE_ERROR;
    }
    ret = read_from_cache(column, data_out, read_len, timeout);
    if (SPI_NAND_RET_OK != ret)
    {
        return READ_FROM_CACHE_ERROR;
    }
    return ret;
}




/**
 * @brief  To erase the Block.
 * @param  row : block address.
 * @retval SPI_NAND_RET_OK in case of success, an error code otherwise.
 */
ReturnType MT29F4G_flash_block_erase(row_address_t row)
{
    uint32_t timeout = 0x00;
    row.page = 0;
    if (!validate_row_address(row))
    {
        return SPI_NAND_RET_BAD_ADDRESS;
    }

    int ret = write_enable(OP_TIMEOUT);
    if (ret)
    {
        return WRITE_ENABLE_ERROR;
    }
    ret = block_erase(row, timeout);
    if(SPI_NAND_RET_OK!=ret){
        return BLOCK_ERASE_ERROR;
    }
    return SPI_NAND_RET_OK;
}

/**
 * @brief Checks if a given block is bad.
 * @note Block operation -- page component of row address is ignored
 * @return SPI_NAND_RET_OK          if good block,
 *         SPI_NAND_RET_BAD_ADDRESS if invalid page or block number
 *         SPI_NAND_RET_INVALID_LEN if provide buffer lenght.
 */
ReturnType MT29F4G_flash_is_bad_block(row_address_t row, bool *is_bad)
{
    uint8_t bad_block_mark[6];
    int ret = MT29F4G_flash_page_read(row, 4096, bad_block_mark, sizeof(bad_block_mark));
    if (ret)
    {
        return FLASH_READ_ERROR;
    }
    if (BAD_BLOCK_MARK == bad_block_mark[0] || BAD_BLOCK_MARK == bad_block_mark[5])
    {
        *is_bad = true;
    }
    else
    {
        *is_bad = false;
    }
    return SPI_NAND_RET_OK;
}



/**
 * @brief  To mark the bad block.
 * @param  row : block address.
 * @retval SPI_NAND_RET_OK in case of success, an error code otherwise.
 */
ReturnType MT29F4G_flash_flashblock_mark_bad(row_address_t row)
{
    ReturnType ret = 0;
    uint8_t bad_block_mark[2] = {BAD_BLOCK_MARK, BAD_BLOCK_MARK};
    ret = MT29F4G_flash_write_page(row, SPI_NAND_PAGE_SIZE, bad_block_mark, sizeof(bad_block_mark));
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }

    return SPI_NAND_RET_OK;
}

/**
 * @brief
 * @param
 * @param
 * @retval SPI_NAND_RET_OK in case of success, an error code otherwise.
 */
ReturnType MT29F4G_flash_flashpage_is_free(row_address_t row, bool *is_free)
{

    uint32_t comp_word = 0xffffffff;

    int ret = MT29F4G_flash_page_read(row, 0x00, page_main_and_oob_buffer, sizeof(page_main_and_oob_buffer));
    if (ret)
    {
        return FLASH_READ_ERROR;
    }
    *is_free = true;

    for (int i = 0; i < sizeof(page_main_and_oob_buffer); i += sizeof(comp_word))
    {
        if (0 != memcmp(&comp_word, &page_main_and_oob_buffer[i], sizeof(comp_word)))
        {
            *is_free = false;
            break;
        }
    }
    return SPI_NAND_RET_OK;
}

ReturnType MT29F4G_flash_page_copy(row_address_t src, row_address_t dest)
{
    uint32_t timeout = 0;
    uint8_t dummy_byte = 0;
    if (!validate_row_address(src) ||
        !validate_row_address(dest))
    {
        return SPI_NAND_RET_BAD_ADDRESS;
    }

    int ret = page_read(src, OP_TIMEOUT);
    if (SPI_NAND_RET_OK != ret)
    {
        return ret;
    }

    ret = write_enable(timeout);
    if (SPI_NAND_RET_OK != ret)
    {
        return WRITE_ENABLE_ERROR;
    }
    ret = program_load_random_data(0, &dummy_byte, 0, timeout);
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }

    ret = program_execute(dest, timeout);
    if (SPI_NAND_RET_OK != ret)
    {
        return PROGRAM_EXECUTE_ERROR;
    }
    return SPI_NAND_RET_OK;
}

ReturnType MT29F4G_flash_chip_erase(){
    bool is_bad;
    for (int i = 0; i < SPI_NAND_BLOCKS_PER_LUN + 4; i++) {
        // get bad block flag
        row_address_t row = {.block = i, .page = 0};
        int ret = MT29F4G_flash_is_bad_block(row, &is_bad);
        if (SPI_NAND_RET_OK != ret) return ret;

        // erase if good block
        if (!is_bad) {
            int ret = MT29F4G_flash_block_erase(row);
            if (SPI_NAND_RET_OK != ret) return ret;
        }
        else{
            LOG_INF("bad block is %d\n",i);
            MT29F4G_flash_block_erase(row);
        }
    }

    // if we made it here, nothing returned a bad status
    return SPI_NAND_RET_OK;

}
// Implement function prototypes declared in the header file

/**
 * @}
 */
