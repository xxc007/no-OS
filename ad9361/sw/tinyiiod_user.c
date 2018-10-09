/*
 * libtinyiiod - Tiny IIO Daemon Library
 *
 * Copyright (C) 2016 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "tinyiiod.h"
#include "tinyiiod_user.h"
#include "adc_core.h"
#include "parameters.h"
#include "xil_io.h"
#include "ad9361_api.h"


static unsigned int addr_to_read;
static uint32_t channel_mask;
extern struct ad9361_rf_phy *ad9361_phy;
extern struct adc_state adc_st;
int einval(void){
	volatile int x=3;
	x++;
	return 22;}
int enoent(void){
	volatile int x=3;
	x++;
	return 2;
}
/***********************************************************************************************************************
* Function Name: read_data
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void read_data(char *buf, size_t len)
{
	uint8_t i, ret;
	ret = read(0, buf, len);

	if (ret < len)
		read(0, buf, len);
//	for(i = 0; i < len; i++)
//		*(buf + i) = inbyte();
}

/***********************************************************************************************************************
* Function Name: write_data
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void write_data(const char *buf, size_t len)
{
	int i;

	for ( i = 0; i < len; i++)
		outbyte(buf[i]);
}

/***********************************************************************************************************************
* Function Name: strequal
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static bool strequal(const char *str1, const char *str2)
{
	return !strcmp(str1, str2);
}

/***********************************************************************************************************************
* Function Name: u_to_string
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static ssize_t u_to_string(char *buf, size_t len, unsigned int value)
{
	return (ssize_t) snprintf(buf, len, "%u", value);
}

static ssize_t i_to_string(char *buf, size_t len, int value)
{
	return (ssize_t) snprintf(buf, len, "%d", value);
}
/***********************************************************************************************************************
* Function Name: dev_is_temp_module
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static bool dev_is_ad9361_module(const char *device)
{
	return strequal(device, "temp_module")
			|| strequal(device, "iio:device0")
			|| strequal(device, "iio:device1")
			|| strequal(device, "iio:device2")
			|| strequal(device, "iio:device3")
			|| strequal(device, "iio:device4");
}

/***********************************************************************************************************************
* Function Name: read_value
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static long read_value(const char *str)
{
	char *end;
	int32_t value = strtol(str, &end, 0);

	if (end == str)
		return -EINVAL;
	else
		return value;
}


/***********************************************************************************************************************
* Function Name: read_value
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static unsigned long read_ul_value(const char *str)
{
	char *end;
	uint32_t value = strtoul(str, &end, 0);

	if (end == str)
		return -EINVAL;
	else
		return value;
}

/***********************************************************************************************************************
* Function Name: write_reg
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static int write_reg(unsigned int addr, uint32_t value)
{
	return -EPERM;
}

/***********************************************************************************************************************
* Function Name: read_reg
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static int read_reg(unsigned int addr, uint32_t *value)
{

	return 0;
}

/***********************************************************************************************************************
* Function Name: read_attr
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static ssize_t read_attr(const char *device, const char *attr,
		char *buf, size_t len, bool debug)
{

	return -ENOENT;
}

/***********************************************************************************************************************
* Function Name: write_attr
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static ssize_t write_attr(const char *device, const char *attr,
		const char *buf, size_t len, bool debug)
{


	return -ENOENT;
}

/***********************************************************************************************************************
* Function Name: ch_read_attr
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static ssize_t ch_read_attr(const char *device, const char *channel,
		bool ch_out, const char *attr, char *buf, size_t len)
{
	int32_t temp;

	if (!dev_is_ad9361_module(device))
				return -ENODEV;

	if (strequal(attr, "Sensor")) {
				ad9361_get_temperature(ad9361_phy, &temp);
				return (ssize_t) snprintf(buf, len, "%d", temp);
			}
	if (strequal(attr, "voltage0")) {
		return (ssize_t) snprintf(buf, len, "%d", 2);
	}
	if (strequal(attr, "calibscale")) {
			return (ssize_t) snprintf(buf, len, "%d", "66");
	}
	if (strequal(attr, "calibphase")) {
			return (ssize_t) snprintf(buf, len, "%d", 2);
	}

	if (strequal(attr, "sampling_frequency")) {
		uint32_t sampling_freq_hz;
		ad9361_get_rx_sampling_freq (ad9361_phy, &sampling_freq_hz);
			return (ssize_t) snprintf(buf, len, "%d", sampling_freq_hz);
	}
	if (strequal(attr, "filter_fir_en")) {
		uint8_t en_dis;
		ad9361_get_rx_fir_en_dis (ad9361_phy, &en_dis);
		return (ssize_t) snprintf(buf, len, "%d", en_dis);
	}

	if (strequal(attr, "sampling_frequency_available")) {
			return (ssize_t) snprintf(buf, len, "%d", 2);
	}
	return -ENOENT;
}

/***********************************************************************************************************************
* Function Name: ch_write_attr
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static ssize_t ch_write_attr(const char *device, const char *channel,
		bool ch_out, const char *attr, const char *buf, size_t len)
{

	if (!dev_is_ad9361_module(device))
			return -ENODEV;
	/* We have no output channels */
	if (ch_out) {

	}
	else { //input channel attributes
		if (strequal(attr, "sampling_frequency")) {
			uint32_t sampling_freq_hz = read_ul_value(buf);
			ad9361_set_rx_sampling_freq (ad9361_phy, sampling_freq_hz);
				return len;
		}
		if (strequal(attr, "filter_fir_en")) {
			uint8_t en_dis = read_ul_value(buf);
			ad9361_set_rx_fir_en_dis (ad9361_phy, en_dis);
			return len;
		}

	}


	return -ENOENT;
}

/***********************************************************************************************************************
* Function Name: open_dev
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static int open_dev(const char *device, size_t sample_size, uint32_t mask)
{
	if (!dev_is_ad9361_module(device))
		return -ENODEV;

	if (mask & ~0x3)
		return -ENOENT;

	channel_mask = mask;
	return 0;
}

/***********************************************************************************************************************
* Function Name: close_dev
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static int close_dev(const char *device)
{
	return dev_is_ad9361_module(device) ? 0 : -ENODEV;
}

/***********************************************************************************************************************
* Function Name: r_uart1_callback_receiveend
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static int get_mask(const char *device, uint32_t *mask)
{
	if (!dev_is_ad9361_module(device))
		return -ENODEV;

	*mask = channel_mask;
	return 0;
}

/***********************************************************************************************************************
* Function Name: read_dev
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static ssize_t read_dev(const char *device, char *buf, size_t bytes_count)
{
	int i, sampleSize;

	if (!dev_is_ad9361_module(device))
			return -ENODEV;

	if(adc_st.rx2tx2)
		{
			sampleSize = bytes_count / 8;
		}
		else
		{
			sampleSize = bytes_count / 4;
		}

	adc_capture(sampleSize, ADC_DDR_BASEADDR);
	Xil_DCacheInvalidateRange(ADC_DDR_BASEADDR,	bytes_count);

	for ( i = 0; i < bytes_count; i++)
		buf[i] = Xil_In8(ADC_DDR_BASEADDR + i);

	return bytes_count;
}

/***********************************************************************************************************************
* Name: tinyiiod_ops
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
const struct tinyiiod_ops ops = {
	.read = read_data,
	.write = write_data,

	.read_attr = read_attr,
	.write_attr = write_attr,
	.ch_read_attr = ch_read_attr,
	.ch_write_attr = ch_write_attr,

	.open = open_dev,
	.close = close_dev,
	.get_mask = get_mask,

	.read_data = read_dev,
};
