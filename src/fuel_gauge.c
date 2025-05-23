/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm1300_charger.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "nrf_fuel_gauge.h"


#define SHIP_BASE 0x0BU
#define SHIP_OFFSET_HIBERNATE 0x00U

/* nPM1300 CHARGER.BCHGCHARGESTATUS.CONSTANTCURRENT register bitmask */
#define NPM1300_CHG_STATUS_CC_MASK BIT(3)

#define MAX_CHG_CURRENT_MA 50

static float max_charge_current;
static float term_charge_current;
static int64_t ref_time;

static const struct battery_model battery_model = {
#include "battery_model.inc"
};

int npm1300_enter_ship_mode(const struct device *dev, uint32_t time_ms)
{
    int ret = 0;
    
    if (dev == NULL) {
        return -EINVAL;
    }
    
    /* If a wake-up timer is requested, configure it first */
    if (time_ms > 0) {
        ret = mfd_npm1300_set_timer(dev, time_ms);
        if (ret != 0) {
            printk("Failed to set timer for Ship mode: %d\n", ret);
            return ret;
        }
    }
    
    /* Enter Ship mode */
    ret = mfd_npm1300_reg_write(dev, SHIP_BASE, SHIP_OFFSET_HIBERNATE, 1U);
    if (ret != 0) {
        printk("Failed to enter Ship mode: %d\n", ret);
        return ret;
    }
    
    return 0;
}

// int mfd_npm1300_hibernate(const struct device *dev, uint32_t time_ms)
// {
	

	

// 	return mfd_npm1300_reg_write(dev, SHIP_BASE, SHIP_OFFSET_HIBERNATE, 1U);
// }

int set_charge_current(const struct device *charger, int current_ma)
{
	struct sensor_value value;
	int ret;

	if (current_ma > MAX_CHG_CURRENT_MA)
	{
		current_ma = MAX_CHG_CURRENT_MA;
	}

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
	// k_msleep(5000);

	// value.val1 = current_ma/4;
	// value.val2 = current_ma%2==0 ? 0 : 1;
	value.val1 = current_ma / 1000; // Millions part
	value.val2 = (current_ma % 1000) * 1000;
	ret = sensor_attr_set(charger, SENSOR_CHAN_CURRENT,
						  SENSOR_ATTR_CONFIGURATION, &value);

	if (ret < 0)
	{
		printk("Failed to set VBUS current limit (%d mA)\n", current_ma);
		return ret;
	}

	value.val1 = 1;
	value.val2 = 0;

	ret = sensor_attr_set(charger, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
						  SENSOR_ATTR_CONFIGURATION, &value);

	if (ret < 0)
		return ret;

	printk("Requested VBUS current limit: %d mA\n", current_ma);

	/* 4) Read back VBUS limit active flag */
	struct sensor_value lim;
	ret = sensor_attr_get(charger,
						  SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT,
						  SENSOR_ATTR_CONFIGURATION,
						  &lim);
	if (ret == 0)
	{
		printk("VBUS current-limit active? %s\n",
			   lim.val1 ? "yes" : "no");
	}

	return ret;
}

static int read_sensors(const struct device *charger,
						float *voltage, float *current, float *temp, int32_t *chg_status)
{
	struct sensor_value value;
	int ret;

	ret = sensor_sample_fetch(charger);
	if (ret < 0)
	{
		return ret;
	}

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &value);
	*voltage = (float)value.val1 + ((float)value.val2 / 1000000);

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_TEMP, &value);
	*temp = (float)value.val1 + ((float)value.val2 / 1000000);

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &value);
	*current = (float)value.val1 + ((float)value.val2 / 1000000);

	sensor_channel_get(charger, SENSOR_CHAN_NPM1300_CHARGER_STATUS, &value);
	*chg_status = value.val1;

	return 0;
}

int fuel_gauge_init(const struct device *charger)
{
	struct sensor_value value;
	struct nrf_fuel_gauge_init_parameters parameters = {
		.model = &battery_model,
		.opt_params = NULL,
	};
	int32_t chg_status;
	int ret;

	printk("nRF Fuel Gauge version: %s\n", nrf_fuel_gauge_version);

	ret = read_sensors(charger, &parameters.v0, &parameters.i0, &parameters.t0, &chg_status);
	if (ret < 0)
	{
		return ret;
	}

	/* Store charge nominal and termination current, needed for ttf calculation */
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT, &value);
	max_charge_current = (float)value.val1 + ((float)value.val2 / 1000000);
	term_charge_current = max_charge_current / 10.f;

	nrf_fuel_gauge_init(&parameters, NULL);
	set_charge_current(charger, MAX_CHG_CURRENT_MA);

	ref_time = k_uptime_get();

	return 0;
}

int fuel_gauge_update(const struct device *charger, bool vbus_connected)
{
	float voltage;
	float current;
	float temp;
	float soc;
	float tte;
	float ttf;
	float delta;
	int32_t chg_status;
	bool cc_charging;
	int ret;

	ret = read_sensors(charger, &voltage, &current, &temp, &chg_status);
	if (ret < 0)
	{
		printk("Error: Could not read from charger device\n");
		return ret;
	}

	cc_charging = (chg_status & NPM1300_CHG_STATUS_CC_MASK) != 0;
	if (chg_status)
	{
		printk("Charging: Yes\n");
	}
	else
	{
		printk("Charging: No\n");
	}

	delta = (float)k_uptime_delta(&ref_time) / 1000.f;

	soc = nrf_fuel_gauge_process(voltage, current, temp, delta, vbus_connected, NULL);
	tte = nrf_fuel_gauge_tte_get();
	ttf = nrf_fuel_gauge_ttf_get(cc_charging, -term_charge_current);

	printk("V: %.3f, I: %.3f, T: %.2f, ", (double)voltage, (double)current, (double)temp);
	printk("SoC: %.2f, TTE: %.0f, TTF: %.0f\n", (double)soc, (double)tte, (double)ttf);

	return 0;
}


