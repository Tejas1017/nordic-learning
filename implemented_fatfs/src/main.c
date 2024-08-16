/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "main.h"
#include "ff.h"
#include "ffconf.h"
#include "mem.h"
#include "shell.h"
/** @brief Register the module for logging. */
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
FATFS fs; // file system object

int main(void)
{

     
        nrf_gpio_cfg_output(FLASH_TRIGGER);
        nrf_gpio_pin_set(FLASH_TRIGGER);

        //    k_msleep(2000);
        nrf_gpio_cfg_input(FLASH_WP_PIN, NRF_GPIO_PIN_NOPULL);

        /* Hold cpin */
        nrf_gpio_cfg_output(FLASH_HOLD_PIN);
        nrf_gpio_pin_set(FLASH_HOLD_PIN);

        LOG_DBG("spi test started\r\n");
        gpio_pin_configure(gpio0_dev, GPIO0_CS, GPIO_OUTPUT);
        gpio_pin_set(gpio0_dev, GPIO0_CS, 1);

        FRESULT res = f_mount(&fs, "", 1);
        if (FR_OK == res)
        {
                shell_prints_line("f_mount succeeded!");
                FIL fil;
                FRESULT res;
                UINT bw;
                UINT br;

                // Create a new file
                res = f_open(&fil, "tejas1.txt", FA_CREATE_ALWAYS | FA_WRITE);
                if (res == FR_OK)
                {
                        // Write data to the file       
                        char data[200] = "tejas vyas";
                        char read_buffer[200] = "";
                        res = f_write(&fil, data, sizeof(data), &bw);
                        if (res == FR_OK)
                        {
                                printf("File written successfully.\n");
                        }
                        else
                        {
                                printf("Failed to write file. Error code: %d\n", res);
                        }
                        f_close(&fil);

                        // Open the file for reading
                        res = f_open(&fil, "tejas1.txt", FA_READ);
                        if (res != FR_OK)
                        {
                                printf("Failed to open file for reading. Error code: %d\n", res);
                                return -1;
                        }

                        // Read the content of the file
                        res = f_read(&fil, read_buffer, sizeof(data), &br);
                        if (res == FR_OK)
                        {
                                printf("File read successfully. Data: %s\n", read_buffer);
                        }
                        else
                        {
                                printf("Failed to read file. Error code: %d\n", res);
                        }
                }

                else
                {
                        printf("Failed to create file. Error code: %d\n", res);
                }

                res = f_unlink("tejas1.txt");
                if (res == FR_OK)
                {
                        printf("File deleted successfully.\n");
                }
                else
                {
                        printf("Failed to delete file. Error code: %d\n", res);
                }
        }
        else
        {
                shell_printf_line("f_mount failed, result: %d.", res);
        }

        // if filesystem mount failed due to no filesystem, attempt to make it
        if (FR_NO_FILESYSTEM == res)
        {
                shell_prints_line("No filesystem present. Attempting to make file system..");
                uint8_t *work_buffer = mem_alloc(FF_MAX_SS);
                if (!work_buffer)
                {
                        shell_prints_line("Unable to allocate f_mkfs work buffer. File system not created.");
                }
                else
                {
                        // make the file system
                        res = f_mkfs("", 0, work_buffer, FF_MAX_SS);
                        if (FR_OK != res)
                        {
                                shell_printf_line("f_mkfs failed, result: %d.", res); // fs make failure
                        }
                        else
                        {
                                shell_prints_line("f_mkfs succeeded!"); // fs make success
                                // retry mount
                                res = f_mount(&fs, "", 1);
                                if (FR_OK == res)
                                {
                                        shell_prints_line("f_mount succeeded!");
                                }
                                else
                                {
                                        shell_printf_line("f_mount failed, result: %d.", res);
                                }
                        }

                        mem_free(work_buffer);
                }
        }
        return 0;
}