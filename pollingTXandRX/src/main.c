#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#define UART_DEVICE_NAME DT_LABEL(DT_NODELABEL(uart0))

const struct device *dev;

void main(void)
{
    static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!uart_dev)
    {
        printk("UART: Device driver not found.\n");
        return;
    }
    int i = 0;
    uint8_t rx_buf[64]; // FIFO buffer size
    int rx_data;
    size_t len = 0;

    while (1)
    {
        int read;
        rx_data = uart_fifo_read(uart_dev, &read, 1);
        if (rx_data > 0)
        {
            rx_buf[len++] = read;
            if (rx_buf[len - 1] == '\n' || rx_buf[len - 1] == '\r')
            {
                rx_buf[len - 1] = '\0';
                printk("%s\n", rx_buf);
                // uart_tx(uart_dev, rx_buf, len, 0);
                len = 0;
            }
        }
    }
}