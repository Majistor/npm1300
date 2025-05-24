/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/sys/printk.h>

#include "fuel_gauge.h"

#define SLEEP_TIME_MS 1000

#define SHIP_BASE 0x0BU
#define SHIP_OFFSET_CFGSTROBE 0x01U
#define SHIP_OFFSET_CONFIG 0x04U
#define TASKENTERSHIPMODE 0x02U

static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_pmic));
static const struct device *charger = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_charger));
static const struct device *regulators = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_regulators));
static volatile bool vbus_connected;

static void event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	static int press_t;

	if (pins & BIT(NPM1300_EVENT_VBUS_DETECTED))
	{
		printk("Vbus connected\n");
		vbus_connected = true;
	}

	if (pins & BIT(NPM1300_EVENT_VBUS_REMOVED))
	{
		printk("Vbus removed\n");
		vbus_connected = false;
	}
	// if (press_t == PRESS_MEDIUM_MS)
	// {
	// 	struct sensor_value value;
	// 	int ret;

	// 	// Convert to microamps and set the value
	// 	value.val1 = 1;
	// 	value.val2 = 0;

	// 	ret = sensor_attr_set(charger, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
	// 						  SENSOR_ATTR_CONFIGURATION, &value);

	// 	if (ret < 0)
	// 	{
	// 		return ret;
	// 		printk("Charging current enabled\n");
	// 	}
	// 	if (vbus_connected)
	// 	{
	// 		printk("USB connected\n");
	// 	}
	// 	else
	// 	{
	// 		regulator_enable(regulators);
	// 	}
	// }
	if (pins & BIT(NPM1300_EVENT_SHIPHOLD_PRESS))
	{
		press_t = k_uptime_get();
	}

	if (pins & BIT(NPM1300_EVENT_SHIPHOLD_RELEASE))
	{
		press_t = k_uptime_get() - press_t;
		// regulator_enable(regulators);
	}

	if (press_t > 3000)
		regulator_enable(regulators);
}

int main(void)
{
	int err;

	if (!device_is_ready(pmic))
	{
		printk("Pmic device not ready.\n");
		return 0;
	}

	if (!device_is_ready(charger))
	{
		printk("Charger device not ready.\n");
		return 0;
	}
	if (!device_is_ready(regulators))
	{
		printk("Regulator device not ready.\n");
		return false;
	}

	if (fuel_gauge_init(charger) < 0)
	{
		printk("Could not initialise fuel gauge.\n");
		return 0;
	}

	static struct gpio_callback event_cb;

	gpio_init_callback(&event_cb, event_callback,
					   BIT(NPM1300_EVENT_VBUS_DETECTED) |
						   BIT(NPM1300_EVENT_VBUS_REMOVED) | BIT(NPM1300_EVENT_SHIPHOLD_PRESS) | BIT(NPM1300_EVENT_SHIPHOLD_RELEASE));

	err = mfd_npm1300_add_callback(pmic, &event_cb);
	if (err)
	{
		printk("Failed to add pmic callback.\n");
		return 0;
	}

	/* Initialise vbus detection status. */
	struct sensor_value val;
	int ret = sensor_attr_get(charger, SENSOR_CHAN_CURRENT, SENSOR_ATTR_UPPER_THRESH, &val);

	if (ret < 0)
	{
		return false;
	}

	vbus_connected = (val.val1 != 0) || (val.val2 != 0);

	printk("vbus connected %d", vbus_connected);

	printk("PMIC device ok\n");
	struct sensor_value value;
	// int current_ma = 200; // Set to 0 to disable charging

	value.val1 = 1; // Millions part
	value.val2 = 0;

	ret = sensor_attr_set(charger, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
						  SENSOR_ATTR_CONFIGURATION, &value);

	// // value.val1 = current_ma/4;
	// // value.val2 = current_ma%2==0 ? 0 : 1;
	// value.val1 = 2; // Millions part
	// value.val2 = 0;
	// ret = sensor_attr_set(charger, SENSOR_CHAN_CURRENT,
	// 					  SENSOR_ATTR_CONFIGURATION, &value);

	// if (ret < 0)
	// {
	// 	printk("Failed to set VBUS current limit (%d mA)\n", current_ma);
	// 	return ret;
	// }
	mfd_npm1300_reg_write(pmic, SHIP_BASE, SHIP_OFFSET_CFGSTROBE, 1U);
	// const struct mfd_npm1300_config *config = pmic->config;

	ret = mfd_npm1300_reg_write(pmic, SHIP_BASE, SHIP_OFFSET_CONFIG, 7);
	if (ret < 0)
	{
		return ret;
	}

	// npm1300_enter_ship_mode(*pmic,0);

	uint8_t c = 0;

	while (1)

	{
		c++;
		fuel_gauge_update(charger, vbus_connected);
		k_msleep(SLEEP_TIME_MS);

		printk("c++ %d", c);

		if (c == 10)
		{

			// Convert to microamps and set the value
			value.val1 = 0;
			value.val2 = 0;

			ret = sensor_attr_set(charger, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
								  SENSOR_ATTR_CONFIGURATION, &value);

			if (ret < 0)
			{
				printk("Failed to disable chrager\n");
				return ret;
			}
			printk("Charging current disabled\n");
			if (vbus_connected)
			{
				printk("Ship mode entry not possible with USB connected\n");
			}
			else
			{
				ret = mfd_npm1300_reg_write(pmic, SHIP_BASE, TASKENTERSHIPMODE, 0x01);
				if (ret < 0)
					printk("cannot enter ship mode %d", ret);
				else
					printk("sship mode %d", ret);
			}
		}
	}
}
