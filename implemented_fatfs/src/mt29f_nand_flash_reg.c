#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>
#include "mt29f_nand_flash_reg.h"
#include "mt29f_nand_flash.h"

LOG_MODULE_REGISTER(spi_flash_reg, LOG_LEVEL_INF);

/**
 * @brief  Row address validation.
 * @param  row: The row address to validate.
 * @retval true if valid, false otherwise.
 */
bool validate_row_address(row_address_t row)
{
    if ((row.block > SPI_NAND_MAX_BLOCK_ADDRESS) || (row.page > SPI_NAND_MAX_PAGE_ADDRESS))
    {
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @brief  Column address validation.
 * @param  column: The column address to validate.
 * @retval true if valid, false otherwise.
 */
bool validate_column_address(column_address_t column)
{
    if (column >= (SPI_NAND_PAGE_SIZE + SPI_NAND_OOB_SIZE))
    {
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @brief  Read a feature from the NAND flash.
 * @param  reg: The feature register to read.
 * @param  data_out: The data read from the feature register.
 * @param  timeout: Timeout duration.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType get_feature(uint8_t reg, uint8_t *data_out, uint32_t timeout)
{
    uint8_t tx_data[FEATURE_TRANS_LEN] = {0};
    uint8_t rx_data[FEATURE_TRANS_LEN] = {0};
    tx_data[0] = CMD_GET_FEATURE;
    tx_data[FEATURE_REG_INDEX] = reg;
    struct spi_buf tx_buf = {.buf = tx_data, .len = sizeof(tx_data)};
    struct spi_buf rx_buf = {.buf = rx_data, .len = sizeof(rx_data)};

    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};
    CS_SELECT(GPIO0_CS);

    int ret = spi_transceive(spi0_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);

    CS_DESELECT(GPIO0_CS);
    if (ret < 0)
    {
        LOG_ERR("spi_transceive_dt() failed, err %d", ret);
        return SPI_NAND_RET_BAD_SPI;
    }

    // If good return, write data out
    *data_out = rx_data[FEATURE_DATA_INDEX];

    return SPI_NAND_RET_OK;
}

/**
 * @brief  Write a feature to the NAND flash.
 * @param  reg: The feature register to write to.
 * @param  data: The data to write to the feature register.
 * @param  timeout: Timeout duration.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType set_feature(uint8_t reg, uint8_t data, uint32_t timeout)
{
    uint8_t tx_data[FEATURE_TRANS_LEN] = {0};
    tx_data[0] = CMD_SET_FEATURE;
    tx_data[FEATURE_REG_INDEX] = reg;
    tx_data[FEATURE_DATA_INDEX] = data;
    struct spi_buf tx_buf = {.buf = tx_data, .len = sizeof(tx_data)};
    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};

    CS_SELECT(GPIO0_CS);
    int ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    CS_DESELECT(GPIO0_CS);

    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    return SPI_NAND_RET_OK;
}




/**
 * @brief  To poll the OPERATION IN PROCESS flag of the Status register.
 * @param  status_out : Pointer to the status reg variable.
 * @param  timeout    : timeout value.
 * @retval SPI_NAND_RET_OK in case of success, an error code otherwise.
 */
ReturnType poll_for_oip_clear(feature_reg_status_t *status_out, uint32_t timeout)
{
    unsigned long long start_time = k_uptime_get();
    timeout = timeout + 100;
    unsigned long long get_feature_timeout = k_uptime_get();
    int ret;

    while (k_uptime_get() - start_time < timeout)
    {

        ret = get_feature(FEATURE_REG_STATUS, &status_out->whole, get_feature_timeout);

        if (ret)
        {
            return SPI_NAND_RET_BAD_SPI;
        }

        else if (0 == status_out->OIP)
        {
            return SPI_NAND_RET_OK;
        }
        else if (status_out->OIP)
        {
            continue;
        }
    }
    return SPI_NAND_RET_TIMEOUT;
}




/**
 * @brief  Perform a software reset of the NAND flash.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType reset(void)
{
    int err;
    /* Setup Command */

    uint8_t tx_data[1] = {0};
    tx_data[0] = CMD_RESET;

    struct spi_buf tx_spi_buf = {.buf = tx_data, .len = sizeof(tx_data)};
    struct spi_buf_set spi_tx_buffer_set = {.buffers = &tx_spi_buf, .count = 1};

    CS_SELECT(GPIO0_CS);
    err = spi_write(spi0_dev, &spi_cfg, &spi_tx_buffer_set);
    CS_DESELECT(GPIO0_CS);

    if (err < 0)
    {
        LOG_ERR("Reset failed\r\n");
        return SPI_NAND_RET_BAD_SPI;
    }
    else
    {
        LOG_INF("Successfully reset\r\n");
        return SPI_NAND_RET_OK;
    }

    feature_reg_status_t status;
    int ret = poll_for_oip_clear(&status, OP_TIMEOUT);
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    return SPI_NAND_RET_OK;
}




/**
 * @brief  Read the manufacturer and device ID from the NAND flash.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType whoami(void)    
{
    int err;
    /* Setup Command */
    uint8_t tx_data[READ_ID_TRANS_LEN + 2] = {0};
    uint8_t rx_data[READ_ID_TRANS_LEN + 2] = {0};

    tx_data[0] = 0x9f;

    struct spi_buf tx_spi_buf = {.buf = tx_data, .len = sizeof(tx_data)};
    struct spi_buf_set spi_tx_buffer_set = {.buffers = &tx_spi_buf, .count = 1};

    struct spi_buf rx_spi_buf = {.buf = rx_data, .len = sizeof(rx_data)};
    struct spi_buf_set spi_rx_buffer_set = {.buffers = &rx_spi_buf, .count = 1};

    CS_SELECT(GPIO0_CS);

    err = spi_write(spi0_dev, &spi_cfg, &spi_tx_buffer_set);
    err = spi_read(spi0_dev, &spi_cfg, &spi_rx_buffer_set);

    CS_DESELECT(GPIO0_CS);

    if (err < 0)
    {
        LOG_ERR("Read Id failed: %d\n", err);
        return err;
    }

    LOG_INF("FLASH MFR ID :  0x%x 0x%x\r\n", rx_data[2], rx_data[3]);
    return err;
}




/**
 * @brief  Unlock all blocks in the NAND flash.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType unlock_all_blocks(void)
{
    int ret;
    feature_reg_block_lock_t unlock_all, readback;
    unlock_all.whole = 0;
    readback.whole = 0;
    ret = get_feature(FEATURE_REG_BLOCK_LOCK, &unlock_all.whole, OP_TIMEOUT);
    if (ret)
    {
        return ret;
    }
    unlock_all.TB = 0;
    unlock_all.BP0 = 0;
    unlock_all.BP1 = 0;
    unlock_all.BP2 = 0;
    unlock_all.BP3 = 0;
    ret = set_feature(FEATURE_REG_BLOCK_LOCK, unlock_all.whole, OP_TIMEOUT);
    if (SPI_NAND_RET_OK != ret)
    {
        return SET_FEATURE_ERROR;
    }

    ret = get_feature(FEATURE_REG_BLOCK_LOCK, &readback.whole, OP_TIMEOUT);
    if (SPI_NAND_RET_OK != ret)
    {
        return GET_FEATURE_ERROR;
    }
    return SPI_NAND_RET_OK;
}




/**
 * @brief  Enable ECC on the NAND flash.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType enable_ecc(void)
{
    int ret = 0;
    feature_reg_configuration_t ecc_enable = {.whole = 0};
    ret = get_feature(FEATURE_REG_CONFIGURATION, &ecc_enable.whole, OP_TIMEOUT);
    if (SPI_NAND_RET_OK != ret)
    {
        return GET_FEATURE_ERROR;
    }
    ecc_enable.ECC_EN = 1;
    ret = set_feature(FEATURE_REG_CONFIGURATION, ecc_enable.whole, OP_TIMEOUT);
    if (SPI_NAND_RET_OK != ret)
    {
        return SET_FEATURE_ERROR;
    }

    return SPI_NAND_RET_OK;
}





/**
 * @brief  Get return code from ECC status.
 * @param  status: The status register data read.
 * @retval Return code based on ECC status.
 */
ReturnType get_ret_from_ecc_status(feature_reg_status_t status)
{
    ReturnType ret;

    switch (status.ECCS0_3)
    {
    case ECC_STATUS_NO_ERR:
    case ECC_STATUS_1_3_NO_REFRESH:
        ret = SPI_NAND_RET_OK;
        break;
    case ECC_STATUS_4_6_REFRESH:
    case ECC_STATUS_7_8_REFRESH:
        ret = SPI_NAND_RET_ECC_REFRESH;
        break;
    case ECC_STATUS_NOT_CORRECTED:
    default:
        ret = SPI_NAND_RET_ECC_ERR;
        break;
    }

    return ret;
}





/**
 * @brief  Perform a page read operation on the NAND flash.
 * @param  row: The row address to read from.
 * @param  timeout: Timeout duration.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType page_read(row_address_t row, uint32_t timeout)
{
    uint8_t tx_data[PAGE_READ_TRANS_LEN];
    tx_data[0] = CMD_PAGE_READ;
    tx_data[1] = row.whole >> 16;
    tx_data[2] = row.whole >> 8;
    tx_data[3] = row.whole;
    struct spi_buf tx_buff = {.buf = tx_data, .len = sizeof(tx_buff)};
    struct spi_buf_set tx_buffer_set = {.buffers = &tx_buff, .count = 1};

    CS_SELECT(GPIO0_CS);
    int ret = spi_write(spi0_dev, &spi_cfg, &tx_buffer_set);
    CS_DESELECT(GPIO0_CS);
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_OK;
    }

    feature_reg_status_t status;

    ret = poll_for_oip_clear(&status, timeout);

    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }

    ret = get_ret_from_ecc_status(status);

    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    return SPI_NAND_RET_OK;
}




/**
 * @brief  Read data from the NAND flash cache.
 * @param  col: The column address to read from.
 * @param  data_out: The data read from the cache.
 * @param  read_len: The length of data to read.
 * @param  timeout: Timeout duration.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType read_from_cache(column_address_t col, uint8_t *data_out, size_t read_len, uint32_t timeout)
{
    int ret;
    uint8_t tx_data[READ_FROM_CACHE_TRANS_LEN];
    tx_data[0] = CMD_READ_FROM_CACHE;
    tx_data[1] = col >> 8;
    tx_data[2] = col;
    tx_data[3] = 0;

    struct spi_buf tx_buf = {.buf = tx_data, .len = sizeof(tx_data)};
    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};

    CS_SELECT(GPIO0_CS);
    ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    tx_buf.buf = data_out;
    tx_buf.len = read_len;
    tx_buf_set.buffers = &tx_buf;
    tx_buf_set.count = 1;
    ret = spi_read(spi0_dev, &spi_cfg, &tx_buf_set);
    CS_DESELECT(GPIO0_CS);
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    return SPI_NAND_RET_OK;
}




/**
 * @brief  Perform a program load operation on the NAND flash.
 * @param  column: The column address to load the program.
 * @param  data_in: The data to be loaded.
 * @param  write_len: The length of data to write.
 * @param  timeout: Timeout duration.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType program_load(column_address_t column, uint8_t *data_in, size_t write_len, uint32_t timeout)
{
    uint8_t tx_data[PROGRAM_LOAD_TRANS_LEN];
    int ret;
    tx_data[0] = CMD_PROGRAM_LOAD;
    tx_data[1] = column >> 8;
    tx_data[2] = column;

    
    struct spi_buf tx_buf = {.buf = tx_data, .len = sizeof(tx_data)};


    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf tx_buf1;
    CS_SELECT(GPIO0_CS);
    ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    tx_buf1.buf = data_in;
    tx_buf1.len = write_len;

    tx_buf_set.buffers = &tx_buf1;
    tx_buf_set.count = 1;
    ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    if (SPI_NAND_RET_OK != ret)
    {
        LOG_ERR("writing problem\n");
        return SPI_NAND_RET_BAD_SPI;
    }

    CS_DESELECT(GPIO0_CS);

    return SPI_NAND_RET_OK;
}




/**
 * @brief  Perform a program execute operation on the NAND flash.
 * @param  row: The row address to execute the program.
 * @param  timeout: Timeout duration.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType program_execute(row_address_t row, uint32_t timeout)
{
    uint8_t tx_data[4];
    tx_data[0] = CMD_PROGRAM_EXECUTE;
    tx_data[1] = row.whole >> 16;
    tx_data[2] = row.whole >> 8;
    tx_data[3] = row.whole;
    struct spi_buf tx_buf = {.buf = tx_data, .len = sizeof(tx_data)};
    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
    CS_SELECT(GPIO0_CS);
    int ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    if (ret)
    {
        return -1;
    }

    CS_DESELECT(GPIO0_CS);
    feature_reg_status_t status;

    /* Wait until that operation finishes */
    ret = poll_for_oip_clear(&status, timeout);

    if (ret)
    {
        return SPI_NAND_RET_BAD_SPI; /* If polling failed, return that status */
    }
    else if (status.P_FAIL)
    {
        /**
         *  This bit will also be set if the user attempts to program
         *  a locked or protected region, including the OTP area.
         */
        return STATUS_FAIL; /* Otherwise, check for P_FAIL */
    }
    else
    {
        return SPI_NAND_RET_OK;
    }
}





/**
 * @brief  Enable write operations on the NAND flash.
 * @param  timeout: Timeout duration.
 * @retval 0 if successful, error code otherwise.
 */
ReturnType write_enable(uint32_t timeout)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    // uint8_t buff[2];
    int ret;
    struct spi_buf tx_buf = {.buf = &cmd, .len = 1};
    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};

    CS_SELECT(GPIO0_CS);
    ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    CS_DESELECT(GPIO0_CS);

    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    return SPI_NAND_RET_OK;
}





/**
 * @brief  To erase at the block level.
 * @param  row : row address.
 * @param  timeout : timeout value.
 * @retval SPI_NAND_RET_OK in case of success, an error code otherwise.
 */
ReturnType block_erase(row_address_t row, uint32_t timeout)
{
    uint8_t tx_data[4];
    tx_data[0] = 0xD8;
    tx_data[1] = row.whole >> 16;
    tx_data[2] = row.whole >> 8;
    tx_data[3] = row.whole;
    struct spi_buf tx_buf = {.buf = tx_data, .len = sizeof(tx_data)};
    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};

    CS_SELECT(GPIO0_CS);
    int ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    CS_DESELECT(GPIO0_CS);
    feature_reg_status_t status;
    ret = poll_for_oip_clear(&status, timeout);
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    else if (status.E_FAIL)
    {
        return STATUS_FAIL;
    }
    else
    {
        return SPI_NAND_RET_OK;
    }
}





/**
 * @brief  To write into the cache register for the specific number of bytes.
 * @param  column    : column address
 * @param  data_in   : Pointer to the buffer ,Pointing the data to be written.
 * @param  write_len : size of the data to be written.
 * @param  timeout   : timeout value.
 * @retval SPI_NAND_RET_OK in case of success, an error code otherwise.
 */
ReturnType program_load_random_data(column_address_t column, uint8_t *data_in, size_t write_len, uint32_t timeout)
{
    uint8_t tx_data[PROGRAM_LOAD_RANDOM_DATA_TRANS_LEN];
    tx_data[0] = CMD_PROGRAM_LOAD_RANDOM_DATA;
    tx_data[1] = column >> 8;
    tx_data[2] = column;

    struct spi_buf tx_buf = {.buf = tx_data, .len = sizeof(tx_data)};
    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
    CS_SELECT(GPIO0_CS);
    int ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    if (SPI_NAND_RET_OK != ret)
    {
        return SPI_NAND_RET_BAD_SPI;
    }
    tx_buf.buf = data_in;
    tx_buf.len = write_len;
    tx_buf_set.buffers = &tx_buf;
    tx_buf_set.count = 1;
    ret = spi_write(spi0_dev, &spi_cfg, &tx_buf_set);
    CS_DESELECT(GPIO0_CS);

    return (SPI_NAND_RET_OK == ret) ? SPI_NAND_RET_OK : SPI_NAND_RET_BAD_SPI;
}