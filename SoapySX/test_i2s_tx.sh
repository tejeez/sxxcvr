#!/bin/sh
# SPDX-License-Identifier: MIT
#
# Script for testing I2S before the ALSA interface is implemented in SoapySX.
# Transmit some random noise.
aplay -D "hw:CARD=SX1255,DEV=0" -r 96000 -c 2 -f S32_LE -t raw /dev/urandom
