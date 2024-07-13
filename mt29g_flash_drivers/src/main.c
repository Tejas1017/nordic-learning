/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>



#define FLASH_WP_PIN      NRF_GPIO_PIN_MAP(0, 22) //IO_2
#define FLASH_HOLD_PIN    NRF_GPIO_PIN_MAP(0, 23) //IO_3
#define  MT29F4G01_READ_ID 0x9F
#define CMD_RESET 0xFF     
#define READ_ID_TRANS_LEN 4
#define READ_ID_MFR_INDEX 2
#define READ_ID_DEVICE_INDEX 3
#define FEATURE_TRANS_LEN 3
#define CMD_GET_FEATURE 0x0F
#define FEATURE_REG_INDEX 1
#define FEATURE_DATA_INDEX 2
#define CMD_SET_FEATURE 0x1F
#define FEATURE_REG_BLOCK_LOCK 0xA0
#define FEATURE_REG_CONFIGURATION 0xB0
#define FEATURE_REG_STATUS 0xC0
 #define  FLASH_ON     NRF_GPIO_PIN_MAP(0, 25) //FLASH 

#define SPI_NAND_OOB_SIZE 256
#define SPI_NAND_PAGE_SIZE 4096
#define SPI_NAND_PAGES_PER_BLOCK 64
#define SPI_NAND_BLOCKS_PER_LUN 2048
#define SPI_NAND_MAX_PAGE_ADDRESS (SPI_NAND_PAGES_PER_BLOCK - 1) /* zero-indexed */
#define SPI_NAND_MAX_BLOCK_ADDRESS (SPI_NAND_BLOCKS_PER_LUN - 1) /* zero-indexed */
#define PAGE_READ_TRANS_LEN 4
#define READ_FROM_CACHE_TRANS_LEN 4

//status value
#define ECC_STATUS_NO_ERR 0b000
#define ECC_STATUS_1_3_NO_REFRESH 0b001
#define ECC_STATUS_4_6_REFRESH 0b011
#define ECC_STATUS_7_8_REFRESH 0b101
#define ECC_STATUS_NOT_CORRECTED 0b010

#define LOG_LEVEL LOG_LEVEL_INF
#define MODULE MAIN
LOG_MODULE_REGISTER(MODULE);


//cmd 
#define CMD_PAGE_READ 0x13                /**/
#define CMD_READ_FROM_CACHE 0x03          /**/
#define CMD_WRITE_ENABLE 0x06             /**/
#define CMD_PROGRAM_LOAD 0x02             /**/
#define CMD_PROGRAM_EXECUTE 0x10          /**/
#define CMD_BLOCK_ERASE 0xD8              /**/
#define CMD_PROGRAM_LOAD_RANDOM_DATA 0x84 /**/
#define PROGRAM_LOAD_TRANS_LEN 3


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


typedef union
{
    uint32_t whole;
    struct
    {
        uint32_t page : 6;   /* Valid range 0-63  */
        uint32_t block : 26; /* Valid range 0-2047 */
    };
} row_address_t;


typedef uint16_t column_address_t;


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

static bool validate_row_address(row_address_t row)
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

static bool validate_column_address(column_address_t column)
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


#define SPI0_NODE DT_NODELABEL(spi1)
static const struct device *spi0_dev = DEVICE_DT_GET(SPI0_NODE);

#define GPIO0_NODE DT_NODELABEL(gpio0)
#define GPIO0_CS 30
static const struct device *gpio0_dev = DEVICE_DT_GET(GPIO0_NODE);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

static struct spi_config spi_cfg = {
        .frequency = 8000000,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
        .slave = 0,
};



static int get_feature(uint8_t reg,uint8_t *data_out,uint32_t  timeout){
        uint8_t tx_data[FEATURE_TRANS_LEN] = {0};
        uint8_t rx_data[FEATURE_TRANS_LEN]={0};
        tx_data[0]=CMD_GET_FEATURE;
        tx_data[FEATURE_REG_INDEX]=reg;
       	struct spi_buf tx_buf = {.buf = tx_data, .len = sizeof(tx_data)};
        struct spi_buf rx_buf={.buf=rx_data,.
        len=sizeof(rx_data)};

        struct spi_buf_set tx_buf_set={.buffers=&tx_buf,.count=1};
        struct spi_buf_set rx_buf_set={.buffers=&rx_buf,.count=1};
        gpio_pin_set(gpio0_dev,GPIO0_CS,0);
        int ret = spi_transceive(spi0_dev,&spi_cfg,&tx_buf_set,&rx_buf_set);

        gpio_pin_set(gpio0_dev,GPIO0_CS,1);
        if (ret < 0)
	{
		LOG_INF("spi_transceive_dt() failed, err %d", ret);
		return ret;
	}

	// If good return, write data out
	*data_out = rx_data[FEATURE_DATA_INDEX];

	return ret;
}
static int set_feature(uint8_t reg,uint8_t data,uint32_t timeout){
        uint8_t tx_data[FEATURE_TRANS_LEN]={0};
        tx_data[0]= CMD_SET_FEATURE ;
        tx_data[FEATURE_REG_INDEX]=reg;
        tx_data[FEATURE_DATA_INDEX]=data;
        struct spi_buf tx_buf ={.buf=tx_data,.len=sizeof(tx_data)};
        struct spi_buf_set tx_buf_set ={.buffers=&tx_buf,.count=1};
        
        
        gpio_pin_set(gpio0_dev,GPIO0_CS,0);
        int ret = spi_write(spi0_dev,&spi_cfg,&tx_buf_set);
        gpio_pin_set(gpio0_dev,GPIO0_CS,1);
        
        
        if(ret){
                return -1;
        }
        return 0;

}
static int poll_for_oip_clear(feature_reg_status_t *status_out, uint32_t timeout)
{
    uint32_t get_feature_timeout = 0x00;

    for (;;)
    {
        int ret = get_feature(FEATURE_REG_STATUS, &status_out->whole, get_feature_timeout);

        if ( ret)
        {
            return ret;
        }

        if (0 == status_out->OIP)
        {
            return 0;
        }
    }
}


/**
 * @brief  To reset(software) the chip.
 * @retval SPI_NAND_RET_OK in case of success, an error code otherwise.
 */
static int reset(void)
{
    int err;
    /* Setup Command */
    
    uint8_t tx_data[1] = {0};
        tx_data[0] = 0xff;

        struct spi_buf tx_spi_buf = {
                                        .buf = tx_data, .len = sizeof(tx_data)
        };

        struct spi_buf_set spi_tx_buffer_set = {
                                        .buffers = &tx_spi_buf,
                                        .count = 1,
        };

        gpio_pin_set(gpio0_dev, GPIO0_CS, 0);
        err = spi_write(spi0_dev, &spi_cfg, &spi_tx_buffer_set);
        gpio_pin_set(gpio0_dev, GPIO0_CS, 1);

        if(err < 0 ) { 
                LOG_INF("Reset failed\r\n"); 
                return -1;
        }else{
                LOG_INF("Successfully reset\r\n");
                return 0;
        }
        feature_reg_status_t status;
        int ret = poll_for_oip_clear(&status,3000);
        if(ret){
                return ret;
        }
        return ret;


}

/**
 * @brief  To read Manufacture and Device id.
 * @retval SPI_NAND_RET_OK in case of success, an error code otherwise.
 */
static int read_id(void)
{
        int err;
    /* Setup Command */
    uint8_t tx_data[READ_ID_TRANS_LEN + 2] = {0};
    uint8_t rx_data[READ_ID_TRANS_LEN + 2] = {0};

    tx_data[0] = 0x9f;

        struct spi_buf tx_spi_buf = {
                                        .buf = tx_data, .len = sizeof(tx_data)
        };

        struct spi_buf_set spi_tx_buffer_set = {
                                        .buffers = &tx_spi_buf,
                                        .count = 1,
        };

        struct spi_buf rx_spi_buf = {
                                        .buf = rx_data, .len = sizeof(rx_data)
        };

        struct spi_buf_set spi_rx_buffer_set = {
                                        .buffers = &rx_spi_buf,
                                        .count = 1,
        };
        
        gpio_pin_set(gpio0_dev, GPIO0_CS, 0);
        
        err = spi_write(spi0_dev,&spi_cfg,&spi_tx_buffer_set);
        err = spi_read(spi0_dev,&spi_cfg,&spi_rx_buffer_set);
        
        gpio_pin_set(gpio0_dev, GPIO0_CS, 1);

        if(err < 0 ){
                LOG_INF("Read Id failed: %d\n", err);
                return err;
        }

        LOG_INF("FLASH MFR ID :  0x%x 0x%x\r\n", 
                 rx_data[2], rx_data[3]);
        return err;
}

static int unlock_all_blocks(){
        int ret ;
        feature_reg_block_lock_t unlock_all, readback;
        unlock_all.whole=0;
        readback.whole=0;
        ret = get_feature(FEATURE_REG_BLOCK_LOCK,&unlock_all.whole,3000);
        if(ret){
                return ret;
        }
        unlock_all.TB = 0;
        unlock_all.BP0 = 0;
        unlock_all.BP1 = 0;
        unlock_all.BP2 = 0;
        unlock_all.BP3 = 0;
        ret = set_feature(FEATURE_REG_BLOCK_LOCK,unlock_all.whole,3000);
        if(ret){
                return ret;
        }
 
        ret = get_feature(FEATURE_REG_BLOCK_LOCK,&readback.whole,3000);
        if(ret){
                return ret;
        }
        return 0;
        

}
static int enable_ecc(){
        int ret =0;
        feature_reg_configuration_t ecc_enable = {.whole=0};
        ret = get_feature(FEATURE_REG_CONFIGURATION,&ecc_enable.whole,3000);
        if(ret){
                return ret;
        }
        ecc_enable.ECC_EN=1;
        ret= set_feature(FEATURE_REG_CONFIGURATION,ecc_enable.whole,3000);
        if(ret){
                return ret;
        }

        return 0;

}
int MT29F4G_flash_init(){
        int ret =reset();
        if(ret){
                LOG_INF("unable to reset\n");
                return -1;
        }
        k_msleep(10);
        ret = read_id();
        if(ret){
                LOG_INF("error in reading id\n");
                return -1;
        }
        ret = unlock_all_blocks();
        if(ret){
             return ret;   
        }
        ret = enable_ecc();
        if(ret){
                return ret ;
        }
        return ret;

}

static int get_ret_from_ecc_status(feature_reg_status_t status)
{
    int ret;

    switch (status.ECCS0_3)
    {
    case ECC_STATUS_NO_ERR:
    case ECC_STATUS_1_3_NO_REFRESH:
        ret = 0;
        break;
    case ECC_STATUS_4_6_REFRESH:
    case ECC_STATUS_7_8_REFRESH:
        ret = -6;
        break;
    case ECC_STATUS_NOT_CORRECTED:
    default:
        ret = -7;
        break;
    }

    return ret;
}




int page_read(row_address_t row, uint32_t timeout){
        uint8_t tx_data[PAGE_READ_TRANS_LEN];
        tx_data[0] = CMD_PAGE_READ;
        tx_data[1]=row.whole>>16;
        tx_data[2]=row.whole>>8;
        tx_data[3]=row.whole;
        struct spi_buf tx_buff={.buf=tx_data,.len=sizeof(tx_buff)};
        struct spi_buf_set tx_buffer_set ={.buffers=&tx_buff,.count=1};
        
        
        gpio_pin_set(gpio0_dev, GPIO0_CS, 0);
        int ret = spi_write(spi0_dev,&spi_cfg,&tx_buffer_set);
        gpio_pin_set(gpio0_dev, GPIO0_CS, 1);
        if(ret){
                return -1;
        }
        feature_reg_status_t status; 
        ret = poll_for_oip_clear(&status,timeout);
        if(ret){
                return -1;
        }
        ret =get_ret_from_ecc_status(status);
        return ret;

        
}



static int read_from_cache(column_address_t col,uint8_t *data_out, size_t read_len, uint32_t timeout){
        int ret;
        uint8_t tx_data[READ_FROM_CACHE_TRANS_LEN];
        tx_data[0]=0x0b;
        tx_data[1]=col>>8;
        tx_data[2]=col;
        tx_data[3]=0;

        struct spi_buf tx_buf={.buf=tx_data,.len=sizeof(tx_data)};
        struct spi_buf_set tx_buf_set ={.buffers=&tx_buf, .count=1};
        struct spi_buf rx_buf ={.buf=data_out,.len=read_len};
        struct spi_buf_set rx_buf_set ={.buffers=&rx_buf,.count=1};

        gpio_pin_set(gpio0_dev,GPIO0_CS,0);
        ret=spi_write(spi0_dev,&spi_cfg,&tx_buf_set);
        if(ret){
                return -1;
        }
        tx_buf.buf=data_out;
        tx_buf.len=read_len;
        tx_buf_set.buffers=&tx_buf;
        tx_buf_set.count=1;
        ret = spi_read(spi0_dev,&spi_cfg,&tx_buf_set);
        gpio_pin_set(gpio0_dev,GPIO0_CS,1);
        return ret;
}
/// program load

int program_load(column_address_t column, uint8_t *data_in,
                 size_t write_len, uint32_t timeout)
{
        uint8_t tx_data[PROGRAM_LOAD_TRANS_LEN];
        int ret;
        tx_data[0]=0x02;
        tx_data[1]=column>>8;
        tx_data[2]=column;
        struct spi_buf tx_buf={.buf=tx_data,.len=sizeof(tx_data)};
        struct spi_buf_set  tx_buf_set ={.buffers=&tx_buf,.count=1};
        struct spi_buf tx_buf1;
        gpio_pin_set(gpio0_dev,GPIO0_CS,0);
        ret = spi_write(spi0_dev,&spi_cfg,&tx_buf_set);
        if(ret){
                return -1;
        }
        tx_buf1.buf=data_in;
        tx_buf1.len=write_len;


        tx_buf_set.buffers=&tx_buf1;
        tx_buf_set.count=1;
        ret =spi_write(spi0_dev,&spi_cfg,&tx_buf_set);
        if(ret){
                printk("writing problem\n");
                return -1;
        }


        gpio_pin_set(gpio0_dev,GPIO0_CS,1);

        return ret;

}
static int program_execute(row_address_t row,uint32_t timeout){
        uint8_t tx_data[4];
        tx_data[0] = CMD_PROGRAM_EXECUTE;
        tx_data[1] = row.whole >> 16;
        tx_data[2] = row.whole >> 8;
        tx_data[3] = row.whole;   
        struct spi_buf tx_buf={.buf=tx_data,.len=sizeof(tx_data)};
        struct spi_buf_set tx_buf_set ={.buffers=&tx_buf,.count=1};
        gpio_pin_set(gpio0_dev,GPIO0_CS,0);
        int ret = spi_write(spi0_dev,&spi_cfg,&tx_buf_set);
        if(ret){
                return -1;
        }

        gpio_pin_set(gpio0_dev,GPIO0_CS,1);
        feature_reg_status_t status;

    /* Wait until that operation finishes */
        ret = poll_for_oip_clear(&status, timeout);

        if ( ret)
        {
                return ret; /* If polling failed, return that status */
        }
         else if (status.P_FAIL)
        {
        /**
         *  This bit will also be set if the user attempts to program
         *  a locked or protected region, including the OTP area.
         */
                return -1; /* Otherwise, check for P_FAIL */
        }
        else
        {
                return 0;
        }       
         
}
static int write_enable(uint32_t timeout){
        uint8_t cmd =CMD_WRITE_ENABLE;
        //uint8_t buff[2];
        int ret;
        struct spi_buf tx_buf ={.buf=&cmd, .len=1};
        struct spi_buf_set tx_buf_set ={.buffers=&tx_buf,.count=1};
        gpio_pin_set(gpio0_dev,GPIO0_CS,0);
        ret =spi_write(spi0_dev,&spi_cfg,&tx_buf_set);
        gpio_pin_set(gpio0_dev,GPIO0_CS,1);
        return ret;
}


int MT29F4G01_write_page(row_address_t row, column_address_t column,uint8_t *data_in, size_t write_len){
        /* Input validation */
        uint32_t timeout =0x0;
        if (!validate_row_address(row) || !validate_column_address(column))
        {
                return -1;
        }
        uint16_t max_write_len = (SPI_NAND_PAGE_SIZE + SPI_NAND_OOB_SIZE) - column;
        if (write_len > max_write_len)
        {
                return -1;
        }
        int ret = write_enable(3000);
        if(ret){
                printk("problem in enabling write\n");
                return -1;
        }
        ret = program_load(column,data_in,write_len,timeout);
        if(ret){
                printk("error in program load\n");
                return -1;
        }
        ret = program_execute(row,timeout);
        if(ret){
                printk("error in program execute\n");
                return -1;
        }

}

int MT29F4G01_page_read(row_address_t row, column_address_t column,uint8_t *data_out,size_t read_len){
        uint32_t timeout =0x00;
        if (!validate_row_address(row) || !validate_column_address(column))
        {
                return -1;;
        }
         uint16_t max_read_len = (SPI_NAND_PAGE_SIZE + SPI_NAND_OOB_SIZE) - column;
        if(read_len>max_read_len)
        {
                return -1;
        }
        int ret = page_read(row,3000);
        ret = read_from_cache(column, data_out, read_len, timeout);
        return ret;
}

int main(void)
{
        nrf_gpio_cfg_output(FLASH_ON);
        nrf_gpio_pin_set(FLASH_ON);

    //    k_msleep(2000);
        nrf_gpio_cfg_input(FLASH_WP_PIN, NRF_GPIO_PIN_NOPULL);

        /* Hold pin */
        nrf_gpio_cfg_output(FLASH_HOLD_PIN);
        nrf_gpio_pin_set(FLASH_HOLD_PIN);

        LOG_DBG("spi test started\r\n");
        gpio_pin_configure(gpio0_dev, GPIO0_CS, GPIO_OUTPUT);
        gpio_pin_set(gpio0_dev, GPIO0_CS, 1);

        if(! device_is_ready(spi0_dev))
        {
                LOG_INF("spi0_dev is not ready \r\n");
                return -1;
        }
        MT29F4G_flash_init();
        row_address_t row={.block=5,.page=0};
        column_address_t col=0;
        char data[10];
        char data_to_write[10]="tejas";
        k_msleep(10);
        
        for(int i=0;i<=63;i++){
                row.page=i;
        //        MT29F4G01_write_page(row,col,data_to_write,sizeof(data_to_write));
                MT29F4G01_page_read(row,col,data,sizeof(data));
               
                if(strcmp(data,data_to_write)==0){
                        printk("%d matched\n",i);
                }
                else
                printk("not match\n");
                k_msleep(10);
        }
       
        return 0;
}
