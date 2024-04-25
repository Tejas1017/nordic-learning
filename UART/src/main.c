#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#define LED2 DT_ALIAS(led2)
#define LED1 DT_ALIAS(led1)
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1, gpios);
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static char rx_buf[10] = {0};
char txBuff[20] = "press a or b ";
// event callback function here ->
static void uartCallBack(const struct device *dev, struct uart_event *evt, void *userData)
{
        switch (evt->type)
        {
        case UART_RX_RDY:
        {
                if (evt->data.rx.len == 1)
                {
                        if (evt->data.rx.buf[evt->data.rx.offset] == 'a')
                        {
                                gpio_pin_toggle_dt(&led1);
                        }
                        else if (evt->data.rx.buf[evt->data.rx.offset] == 'b')
                        {
                                gpio_pin_toggle_dt(&led2);
                        }
                }
        }
        break;

        case UART_RX_DISABLED:
        {
                uart_rx_enable(dev, rx_buf, sizeof(rx_buf), 100);
        }
        break;
        }
}
int main()
{
        printk("in main\n");
        if (!device_is_ready(uart))
        {
                printk("uart device is not ready\n");
                return 1;
        }
        if (!device_is_ready(led2.port))
        {
                printk("led2 haveing problem\n");
                return 1;
        }
        if (!device_is_ready(led1.port))
        {
                printk("led1 having problem\n");
                return 1;
        }

        int ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
        if (ret < 0)
        {
                printk("led1 output not set\n");
                return 1;
        }
        ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
        if (ret < 0)
        {
                printk("led2 output not set\n");
                return 1;
        }
        uart_tx(uart, txBuff, sizeof(txBuff), SYS_FOREVER_MS);
        ret = uart_callback_set(uart, uartCallBack, NULL);
        printk("%d\n", ret);
        if (ret)
        {
                printk("this not working\n");
                return 1;
        }

        uart_rx_enable(uart, rx_buf, sizeof(rx_buf), 100);
        while (1)
        {
                // printk)
                k_msleep(100);
        }
}
