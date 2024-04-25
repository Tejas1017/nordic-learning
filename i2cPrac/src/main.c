// #include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>

#define SENSOR_OUTPUT NRF_GPIO_PIN_MAP(0, 26)

void i2c_scan_npm(void)
{
        int err;

        const struct device *i2c_dev = device_get_binding("I2C1");
        if (!i2c_dev)
        {
                printf("I2C device not found\n");
                return;
        }

        printf("Scanning I2C bus...\n");

        for (uint8_t addr = 1; addr <= 127; addr++)
        {
                struct i2c_msg msgs[1];
                uint8_t data = 0;

                /* Send dummy data to probe the device */
                msgs[0].buf = &data;
                msgs[0].len = 1;
                msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

                err = i2c_transfer(i2c_dev, &msgs[0], 1, addr);
                if (err == 0)
                {
                        printf("Found device at address: 0x%02X\n", addr);
                        return ;
                }
                else if (err == -ENODEV)
                {
                        /* Device not found */
                }
                else
                {
                        // printf("Error scanning address 0x%02X: %d\n", addr, err);
                }
        }   printf("device not found\n");
}

void i2c_scan_touch(void)
{
        uint8_t i2c_addr;
        int ret;

        const struct device *i2c_dev = device_get_binding("I2C1");
        if (!i2c_dev)
        {
                printf("Cannot get I2C device\n");
                return;
        }
        nrf_gpio_pin_clear(SENSOR_OUTPUT);
        k_sleep(K_MSEC(2000));
        nrf_gpio_pin_set(SENSOR_OUTPUT);
        k_sleep(K_MSEC(5));

        printf("Scanning I2C bus...\n");
        for (i2c_addr = 0x08; i2c_addr < 0x78; i2c_addr++)
        {
                struct i2c_msg msgs[1];
                uint8_t data = 0; // Dummy data, not used for scanning

                msgs[0].buf = &data;
                msgs[0].len = sizeof(data);
                msgs[0].flags = I2C_MSG_WRITE;

                uint8_t my_addr = (i2c_addr << 1);
                ret = i2c_transfer(i2c_dev, msgs, 1, my_addr);
                if (ret == 0)
                {
                        printf("Found device at address: 0x%02X\n", my_addr);
                        return ;
                }
                else if (ret == -ENODEV)
                {
                        // No ACK received, device not present
                        printf("device is not present\n");
                }
                else
                {
                        //    ..   printf("Error scanning address 0x%02X: %d\n", i2c_addr, ret);
                }

                k_sleep(K_MSEC(5)); // Small delay between scans
        }
             
}

int main(void)
{
        nrf_gpio_cfg_output(SENSOR_OUTPUT);
         i2c_scan_npm();
        //i2c_scan_touch();
        return 0;
}