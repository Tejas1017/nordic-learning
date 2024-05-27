#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>

#define I2C_NODE DT_NODELABEL(hdc2010)

// config reg
#define CONFIG_REG 0X0E
#define MEASUREMENT_CONFIG 0x0F

// who i am reg
#define DEVICE_ID_L 0xFE
#define DEVICE_ID_H 0xFF

// menu id
#define MENU_ID_L 0xFC
#define MENU_ID_H 0xFD

// reading temp reg
#define TEMP_L 0X00
#define TEMP_H 0X01

// init flag
int initFlag = 0;
// init temp flag
int tempFlag = 0;
// init hum flag
int humFlag = 0;

uint8_t config[2] = {CONFIG_REG, 0x80};

// who am i
int readDeviceId(const struct i2c_dt_spec *hdc2010, uint16_t *id, uint16_t *id1)
{
        int ret;
        // first reading device id
        uint8_t reg[2] = {DEVICE_ID_L, DEVICE_ID_H};
        uint8_t device_id[2] = {DEVICE_ID_L, DEVICE_ID_H};
        ret = i2c_write_read_dt(hdc2010, &reg[0], 1, &device_id[0], 1);
        if (ret)
        {
                printk("device id error\n");
                return -1;
        }
        ret = i2c_write_read_dt(hdc2010, &reg[1], 1, &device_id[1], 1);
        if (ret)
        {
                printk("device id error\n");
                return -1;
        }
        *id = (device_id[1] << 8) | device_id[0];
        // second reading menu id
        reg[0] = MENU_ID_L;
        reg[1] = MENU_ID_H;
        ret = i2c_write_read_dt(hdc2010, &reg[0], 1, &device_id[0], 1);
        if (ret)
        {
                printk("menu id error\n");
                return -1;
        }
        ret = i2c_write_read_dt(hdc2010, &reg[1], 1, &device_id[1], 1);
        if (ret)
        {
                printk("menu id error");
                return -1;
        }
        *id1 = (device_id[1] << 8) | device_id[0];
        printk("%x %x device id and menu id\n", *id, *id1);
        return 0;
}
// init
int hdc2010Init(const struct i2c_dt_spec *hdc2010)
{
        int ret;
        if (initFlag)
        {
                return 0;
        }
        ret = i2c_write_dt(hdc2010, config, sizeof(config));
        if (ret != 0)
        {
                printk("soft reseting is fail\n");
                return -1;
        }
        config[1] = 0x70;
        ret = i2c_write_dt(hdc2010, config, sizeof(config));
        if (ret != 0)
        {
                printk("set sampling rate is fail\n");
                return -1;
        }
        /*
        Bits 2 and 1 of the MEASUREMENT_CONFIG register controls
        the measurement mode
        */
        // Upper two bits of the MEASUREMENT_CONFIG register controls the temperature resolution
        // default value is 14
        // Bit 0 of the MEASUREMENT_CONFIG register can be used to trigger measurements

        uint8_t tempVal[2];
        config[0] = MEASUREMENT_CONFIG;
        config[1] = 0X4b;
        ret = i2c_write_dt(hdc2010, config, sizeof(config));
        if (ret != 0)
        {
                printk("setup measurement config failed \n");
                return -1;
        }
        initFlag = 1;
        tempFlag = 1;
        return 0;
}
int hdc2010DeInit()
{
        tempFlag = 0;
        humFlag = 0;
        initFlag = 0;
        return 0;
}

int Hdc2010ReadingTemperature(const struct i2c_dt_spec *hdc2010, int *val)
{
        int ret;
        volatile uint16_t temp;
        float intermediate, value;
        hdc2010Init(hdc2010);
        uint8_t tempReg[2] = {TEMP_L, TEMP_H};
        uint8_t reading[2];
        ret = i2c_write_read_dt(hdc2010, &tempReg[0], 1, &reading[0], 1);
        if (ret)
        {
                printk("failed at reading temperature\n");
                return -1;
        }
        ret = i2c_write_read_dt(hdc2010, &tempReg[1], 1, &reading[1], 1);
        if (ret)
        {
                printk("failed at reading temperature\n");
                return -1;
        }
        temp = (reading[1] << 8) | reading[0];
        intermediate = temp / 65536.0;
        intermediate = intermediate * 165;
        value = intermediate - 40.00;
        *val = value;
        printk("%0.2f C\n", value);
        hdc2010DeInit();
        return 0;
}
int main(void)
{
        int ret;
        static const struct i2c_dt_spec hdc2010 = I2C_DT_SPEC_GET(I2C_NODE);
        if (!device_is_ready(hdc2010.bus))
        {
                printk("device is not ready\n");
                return 0;
        }

        // sensor init part is here
        ret = hdc2010Init(&hdc2010);
        if (ret)
        {
                printk("hdc2010 sensor init failed\n");
                return -1;
        }
        printk("init completed\n");
        int val1, val2;
        readDeviceId(&hdc2010, &val1, &val2);
        int temp;
        while (1)
        {
                Hdc2010ReadingTemperature(&hdc2010, &temp);
                k_msleep(1000);
        }
        return 0;
}
