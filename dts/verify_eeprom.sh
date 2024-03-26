#!/bin/sh
# SPDX-License-Identifier: MIT

# This is run by Makefile to verify EEPROM contents.
# First argument is path to an EEPROM image to compare against.
# EEPFLASH environment variable is path to eepflash.sh.

RESULT=1
TEMP=$(mktemp) || exit $?
${EEPFLASH} -y -r "-f=${TEMP}" -t=24c256 && (
	head -c $(stat -c %s -- "$1") -- ${TEMP} | (
        diff -- "$1" -
        RESULT=$?
        if [ ${RESULT} -eq 0 ]
        then
            echo "EEPROM verification OK."
            echo "Please reboot your Raspberry Pi now."
        else
            echo "EEPROM verification failed. Data read did not match the image file."
            echo "Make sure write protection is disabled (place a jumper between WP pins)"
            echo "and try writing the EEPROM again."
        fi
        exit ${RESULT}
    )
)
RESULT=$?
rm -f -- "${TEMP}"
exit ${RESULT}
