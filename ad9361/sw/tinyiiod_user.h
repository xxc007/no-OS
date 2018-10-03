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

#ifndef TINYIIOD_USER_H
#define TINYIIOD_USER_H

#define USE_LIBIIO   YES

//#define ARRAY_SIZE(v) (sizeof(v) ? sizeof(v) / sizeof((v)[0]) : 0)

static const char * const  xml =
"<?xml version=\"1.0\" encoding=\"utf-8\"?><!DOCTYPE context [<!ELEMENT context "
"(device)*><!ELEMENT device (channel | attribute | debug-attribute)*><!ELEMENT "
"channel (scan-element?, attribute*)><!ELEMENT attribute EMPTY><!ELEMENT "
"scan-element EMPTY><!ELEMENT debug-attribute EMPTY><!ATTLIST context name "
"CDATA #REQUIRED description CDATA #IMPLIED><!ATTLIST device id CDATA "
"#REQUIRED name CDATA #IMPLIED><!ATTLIST channel id CDATA #REQUIRED type "
"(input|output) #REQUIRED name CDATA #IMPLIED><!ATTLIST scan-element index "
"CDATA #REQUIRED format CDATA #REQUIRED scale CDATA #IMPLIED><!ATTLIST "
"attribute name CDATA #REQUIRED filename CDATA #IMPLIED><!ATTLIST "
"debug-attribute name CDATA #REQUIRED>]><context name=\"tiny\" "
"description=\"Tiny IIOD\" >"

"<device id=\"iio:device0\" name=\"ad9361\" >"

"<channel id=\"voltage0\" type=\"input\" >"
"<scan-element index=\"0\" format=\"le:S12/16&gt;&gt;0\" />"
"<attribute name=\"calibphase\" filename=\"in_voltage0_calibphase\" />"
"<attribute name=\"calibbias\" filename=\"in_voltage0_calibbias\" />"
"<attribute name=\"calibscale\" filename=\"in_voltage0_calibscale\" />"
"<attribute name=\"samples_pps\" filename=\"in_voltage_samples_pps\" />"
"<attribute name=\"sampling_frequency\" filename=\"in_voltage_sampling_frequency\" />"
"</channel>"

"<channel id=\"voltage1\" type=\"input\" >"
"<scan-element index=\"1\" format=\"le:S12/16&gt;&gt;0\" />"
"<attribute name=\"calibphase\" filename=\"in_voltage0_calibphase\" />"
"<attribute name=\"calibbias\" filename=\"in_voltage0_calibbias\" />"
"<attribute name=\"calibscale\" filename=\"in_voltage0_calibscale\" />"
"<attribute name=\"samples_pps\" filename=\"in_voltage_samples_pps\" />"
"<attribute name=\"sampling_frequency\" filename=\"in_voltage_sampling_frequency\" />"
"</channel>"


//"<buffer-attribute name=\"watermark\" />"
//"<buffer-attribute name=\"data_available\" />"

//"<channel id=\"voltage0\" name=\"voltage0\" type=\"input\" >"
//"<scan-element index=\"0\" format=\"le:s32/32&gt;&gt;0\" />"
//"<attribute name=\"raw\" />"
//"</channel>"
//
//"<debug-attribute name=\"direct_reg_access\" />"
"</device></context>";

#endif /* TINYIIOD_USER_H */
