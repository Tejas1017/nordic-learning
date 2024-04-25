#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#define ArrSize 65
typedef struct ringBuffer
{
        char ARRdata[ArrSize];
        int head;
        int tail;
} ringBuffer;
void initRingBuffer(ringBuffer *buffer)
{
        buffer->head = 0;
        buffer->tail = 0;
}
void insertData(ringBuffer *buffer, int data)
{
        buffer->ARRdata[buffer->head] = data;
        buffer->head = (buffer->head + 1) % ArrSize;
}
char readFromBuffer(ringBuffer *buffer)
{
        int data = buffer->ARRdata[buffer->tail];
        buffer->tail = (buffer->tail + 1) % ArrSize;
        return data;
}

int main(void)
{
        int len = 0;
        ringBuffer msgBuff;
        initRingBuffer(&msgBuff);
        static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
        if (!uart)
        {
                printk("uart is not ready\n");
                return 0;
        }
        while (1)
        {
                //  printk("in main\n");
                int read;
                int rxData = uart_fifo_read(uart, &read, 1);
                if (rxData > 0)
                {
                        //    printk("in if condition\n");
                        insertData(&msgBuff, read);
                        len++;
                        if (msgBuff.ARRdata[len - 1] == '\n' || msgBuff.ARRdata[len - 1] == '\r')
                        {
                                msgBuff.ARRdata[len - 1] = '\0';
                                printk("%s\n", msgBuff.ARRdata);
                                initRingBuffer(&msgBuff);
                                len = 0;
                        }
                }
        }
        return 0;
}
