/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

LOG_MODULE_REGISTER(adc1, LOG_LEVEL_DBG);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

int main(void)
{
	int err;
	uint32_t count = 0;

	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
		// Optional
		//.calibrate = true,
	};

	if (!adc_is_ready_dt(&adc_channel))
	{
		printk("ADC controller devivce %s not ready", adc_channel.dev->name);
		return 0;
	}

	err = adc_channel_setup_dt(&adc_channel);
	if (err < 0)
	{
		printk("Could not setup channel #%d (%d)", 0, err);
		return 0;
	}

	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err < 0)
	{
		printk("Could not initalize sequnce");
		return 0;
	}
	double v;
	while (1)
	{
		int val_mv;

		err = adc_read(adc_channel.dev, &sequence);
		if (err < 0)
		{
			printk("Could not read (%d)", err);
			continue;
		}

		val_mv = (int)buf;
		printk("ADC reading[%u]: %s, channel %d: Raw: %d\n", count++, adc_channel.dev->name,
			   adc_channel.channel_id, val_mv);
		// adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
		// // v = (double)(val_mv / 4095) * 3.3;
		// float v = val_mv / 1000;
		k_msleep(1000);
		// LOG_INF(" = %f V", v);
	}
	return 0;
}
