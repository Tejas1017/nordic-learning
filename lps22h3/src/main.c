// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/logging/log.h>
// #include <hal/nrf_gpio.h>
// #include <zephyr/drivers/i2c.h>
// #define I2C1_NODE DT_NODELABEL(mysensor)
// char WHO_AM_I = 0xf;
// #define LPS22HB_REG_TEMP_OUT_L 0x2B
// #define LPS22HB_REG_TEMP_OUT_H 0x2C
// #define DELAY 1000
// uint8_t CTRL_REG1 = 0x10;
// #define SENSOR_OUTPUT NRF_GPIO_PIN_MAP(0, 26)

// static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);
// uint8_t temp_reading[2] = {0};
// uint8_t sensor_regs[2] = {LPS22HB_REG_TEMP_OUT_L, LPS22HB_REG_TEMP_OUT_H};
// int main(void)
// {
//         // i2c_scan_touch();

//         if (!device_is_ready(dev_i2c.bus))
//         {
//                 printk("device is not ready\n");
//                 return 1;
//         }
//         uint8_t temp;
//         int r = i2c_write_read_dt(&dev_i2c, &CTRL_REG1, 1, &temp, 1);
//         printk("r value is %d for ctrl_reg is %d", r, temp);
//         if (temp == 0)
//         {
//                 printk("inside if condition\n");
//                 temp = 32;
//                 // setting this bit so 10 hz;;

//                 char config[2] = {CTRL_REG1, temp};
//                 printk("%x\n", temp);
//                 i2c_write_dt(&dev_i2c, config, sizeof(config));
//         }

//         // printk("%d\n", temp >> 4);
//         i2c_write_read_dt(&dev_i2c, &CTRL_REG1, 1, &temp, 1);
//         printk("%d", temp);
//         // while (1)
//         // {
//         //         int ret = i2c_write_read_dt(&dev_i2c, &sensor_regs[0], 1, &temp_reading[0], 1);
//         //         if (ret != 0)
//         //         {
//         //                 printk("can't read from this reg val\n");
//         //         }
//         //         printk("%x  ", temp_reading[0]);
//         //         ret = i2c_write_read_dt(&dev_i2c, &sensor_regs[1], 1, &temp_reading[1], 1);
//         //         if (ret != 0)
//         //         {
//         //                 printk("can't read from this reg val\n");
//         //         }
//         //         printk("%x\n", temp_reading[1]);
//         //         k_msleep(1000);
//         // }

//         return 0;
// }
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#define I2C1_NODE DT_NODELABEL(mysensor)
#define LPS22HB_REG_TEMP_OUT_L 0x2B
#define LPS22HB_REG_TEMP_OUT_H 0x2C
#define DELAY 1000

LOG_MODULE_REGISTER(PRAC_4, LOG_LEVEL_DBG);

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);
struct sensor_value temp;
uint8_t sensor_regs[2] = {LPS22HB_REG_TEMP_OUT_L, LPS22HB_REG_TEMP_OUT_H};
#define ADC_MAX_VALUE 4095
#define V_REF 3.3            // Reference voltage for the ADC
#define TEMP_MIN_VOLTAGE 0.1 // Example: Minimum voltage corresponding to minimum temperature
#define TEMP_MAX_VOLTAGE 3.0 // Example: Maximum voltage corresponding to maximum temperature
#define TEMP_MIN_CELSIUS -40 // Example: Minimum temperature in Celsius
#define TEMP_MAX_CELSIUS 125 //
float adc_to_temperature(uint16_t adc_value)
{
        // Convert ADC value to voltage
        float voltage = (float)adc_value / ADC_MAX_VALUE * V_REF;

        // Linear interpolation to find temperature
        float temp_range = TEMP_MAX_CELSIUS - TEMP_MIN_CELSIUS;
        float voltage_range = TEMP_MAX_VOLTAGE - TEMP_MIN_VOLTAGE;
        float temp_celsius = ((voltage - TEMP_MIN_VOLTAGE) / voltage_range) * temp_range + TEMP_MIN_CELSIUS;

        return temp_celsius;
}
int main(void)
{

        if (!device_is_ready(dev_i2c.bus))
        {
                printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
                return 0;
        }
        unsigned char whoIamI[1] = {0xf};
        uint8_t valFromwhoIami;
        int ret = i2c_write_read_dt(&dev_i2c, whoIamI, 1, &valFromwhoIami, 1);
        printk("%x\n", valFromwhoIami);
        ;
        while (1)
        {
                ret = i2c_write_read_dt(&dev_i2c, &sensor_regs[0], 1, &temp.val1, 1);
                if (ret != 0)
                {
                        printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr, sensor_regs[0]);
                }
                //   printk(" temp_red = %d \n\r",  temp.val1);
                ret = i2c_write_read_dt(&dev_i2c, &sensor_regs[1], 1, &temp.val2, 1);
                if (ret != 0)
                {
                        printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr, sensor_regs[1]);
                }
                //   printk(" temp_red = %d \n\r",  temp.val2);
                int val = temp.val2 << 8 | temp.val1;

                double cTemp = (double)val / 100;
                printk("%lf\n", cTemp);
                k_msleep(DELAY);
        }
        return 0;
}