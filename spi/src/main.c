#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

// #define CHIP_ID_REG 0x02
#define READ_CMD 0x0b
// static const struct spi_config spi_cfg = {
// 	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
// 				 SPI_MODE_CPOL | SPI_MODE_CPHA,
// 	.frequency = 4000000,
// 	.slave = 0,
// };
#define SPI1_NODE DT_NODELABEL(spi2)
static const struct device *spi_dev = DEVICE_DT_GET(SPI1_NODE);

#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB
struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(adxl362), SPIOP, 0);

void adxl326Read(struct spi_dt_spec *spi, int *data, int reg, int size)
{
	uint8_t txBuff[3] = {READ_CMD, reg, 0xff};
	uint8_t rxBuffer[4];
	// tx buff
	struct spi_buf txSpiBuf = {.buf = (void *)&txBuff, .len = 4};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &txSpiBuf, .count = 1};

	// rx part
	struct spi_buf rxSpiBuf = {.buf = rxBuffer, .len = 4};
	struct spi_buf_set rx_spi_buf_set = {.buffers = &rxSpiBuf, .count = 1};

	int err = spi_transceive_dt(spi, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err)
	{
		printk("err in reading data\n");
	}
	// for (int i = 0; i < 4; i++
	// {
	// 	printk("%x \n", rxBuffer[i]);
	// }
	printk("data is %d\n", rxBuffer[2]);
}

int main(void)
{
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB
	struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(adxl362), SPIOP, 0);
	int data[4];
	int reg = 0x02;
	uint8_t tx_buf[] = {0x0A, (reg & 0x7F)};
	struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

	int err = spi_write_dt(&spispec, &tx_spi_buf_set);
	if (err)
	{
		printk("writing problem \n");
		return 1;
	}
	uint8_t rxBuff[2];
	struct spi_buf rx_spi_buf = {.buf = rxBuff, .len = sizeof(rxBuff)};
	struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count = 1};
	// spi_read_dt(&spispec,&rx_spi_buf_set);
	// adxl326Read(&spispec,data,0x02,4);
	// while (1)
	// {
	// 	k_sleep(K_MSEC(1000));
	// 	adxl326Read(&spispec, data, 0x14, 4);
	// }
}
