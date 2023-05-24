#!/bin/sh
# SPDX-License-Identifier: MIT
#
# Script for testing I2S before the ALSA interface is implemented in SoapySX.
arecord -D "hw:CARD=SX1255,DEV=1" -r 96000 -c 2 -f S32_LE -t raw rx_test.raw
