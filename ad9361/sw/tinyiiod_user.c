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
#include "dac_core.h"
#include "parameters.h"
#include "xil_io.h"
#include "ad9361_api.h"
#include "serial.h"
#include "xil_cache.h"

static unsigned int addr_to_read;
static uint32_t request_mask;
// mask for cf-ad9361-lpc 0x0F, it has 4 channels
static uint32_t input_channel_mask = 0x0F;
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
static int read_data(char *buf, size_t len)
{
	return serial_read_line(buf, len);
}

static int read_nonbloking(char *buf, size_t len)
{
	return serial_read_nonblocking(buf, len);
}

static int read_wait(size_t len)
{
	return serial_read_wait(len);
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
				return (ssize_t) snprintf(buf, len, "%d", (int)temp);
			}
	if (strequal(attr, "voltage0")) {
		return (ssize_t) snprintf(buf, len, "%d", 2);
	}
	if (strequal(attr, "calibscale")) {
			return (ssize_t) snprintf(buf, len, "%d", 66);
	}
	if (strequal(attr, "calibphase")) {
			return (ssize_t) snprintf(buf, len, "%d", 2);
	}

	if (strequal(attr, "sampling_frequency")) {
		uint32_t sampling_freq_hz;
		ad9361_get_rx_sampling_freq (ad9361_phy, &sampling_freq_hz);
			return (ssize_t) snprintf(buf, len, "%d", (int)sampling_freq_hz);
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

	if (mask & ~input_channel_mask)
		return -ENOENT;

	request_mask = mask;
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

	//*mask = request_mask;
	*mask = input_channel_mask; // this way client has to do demux of data
	return 0;
}

static ssize_t write_dev(const char *device, const char *buf, size_t bytes_count)
{
//	todo
//	int i, sampleSize;
//
//	if (!dev_is_ad9361_module(device))
//			return -ENODEV;
//
//	if(adc_st.rx2tx2)
//		{
//			sampleSize = bytes_count / 8;
//		}
//		else
//		{
//			sampleSize = bytes_count / 4;
//		}
//
//	adc_capture(sampleSize, ADC_DDR_BASEADDR);
//	Xil_DCacheInvalidateRange(ADC_DDR_BASEADDR,	bytes_count);
//
//	for ( i = 0; i < bytes_count; i++)
//		buf[i] = Xil_In8(ADC_DDR_BASEADDR + i);
	dac_write_buffer(device, 0, 0);
	return bytes_count;
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
	//communication
	.read = read_data,
	.read_nonbloking = read_nonbloking,
	.read_wait = read_wait,
	.write = write_data,

	//device operations
	.read_attr = read_attr,
	.write_attr = write_attr,
	.ch_read_attr = ch_read_attr,
	.ch_write_attr = ch_write_attr,
	.read_device = read_dev,
	.write_device = write_dev,

	//
	.open = open_dev,
	.close = close_dev,
	.get_mask = get_mask,
};
