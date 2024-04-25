#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
int main(void)
{
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&led, 1);

        pm_device_init_off(&led);
        while (1)
        {

                int ret = pm_device_action_run(&led, PM_DEVICE_ACTION_TURN_ON);
                if (ret < 0)
                {
                        printk("Failed to power on my led\n");
                        return -1;
                }
                printk("My device powered on\n");
                k_msleep(1000);
                ret = pm_device_action_run(&led, PM_DEVICE_ACTION_TURN_OFF);
                if (ret < 0)
                {
                        printk("Failed to power off my device\n");
                        return -1;
                }
                k_msleep(1000);
                printk("my device is powered off\n");
                }
}