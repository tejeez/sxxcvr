#!/bin/sh
set -e

DTBO_PATH=/boot/overlays/sx1255.dtbo
CONFIG_PATH=/boot/config.txt

echo "Compiling and installing DTBO at ${DTBO_PATH}"
dtc -Wno-unit_address_vs_reg sx1255_raspberrypi.dts | sudo tee "${DTBO_PATH}" >/dev/null

echo "Checking ${CONFIG_PATH}"
if grep -qx "dtoverlay=sx1255" "${CONFIG_PATH}"
then
	echo "Looks like SX1255 configuration was there already."
else
	echo "Adding SX1255 configuration in ${CONFIG_PATH}"
	echo "\n# SX1255 configuration\n[all]\ndtparam=i2s=on\ndtparam=spi=on\ndtoverlay=sx1255\n" | sudo tee -a "${CONFIG_PATH}"
fi
echo "Done. Please reboot your Raspberry Pi now."
