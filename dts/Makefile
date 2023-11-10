# SPDX-License-Identifier: MIT

BUILD_DIR=build
EEPMAKE=eepmake
EEPFLASH=eepflash.sh

# If the HAT currently connected already has an UUID, keep it unmodified.
# Otherwise let eepmake generate an UUID.
HAT_UUID=$(file < /proc/device-tree/hat/uuid)
ifeq ($(HAT_UUID),)
HAT_UUID:=00000000-0000-0000-0000-000000000000
endif

# If HAT hardware version is already known, keep it unmodified.
HAT_VERSION=$(file < /proc/device-tree/hat/product_ver)
ifeq ($(HAT_VERSION),)
# Default if hardware version is not specified
HAT_VERSION:=0x0101
endif

ifeq ($(HAT_VERSION),0x0100)
HAT_TX_CONTROL_PIN:=12
HAT_RX_CONTROL_PIN:=13
else
HAT_TX_CONTROL_PIN:=22
HAT_RX_CONTROL_PIN:=23
endif

define HAT_EEPROM_SETTINGS
product_uuid $(HAT_UUID)
product_id 0x1255
product_ver $(HAT_VERSION)
vendor OH2EAT
product SXceiver
gpio_drive 0
gpio_slew 0
gpio_hysteresis 1
back_power 0
setgpio $(HAT_TX_CONTROL_PIN) OUTPUT DOWN
setgpio $(HAT_RX_CONTROL_PIN) OUTPUT DOWN
endef

all: $(BUILD_DIR)/eeprom.bin

$(BUILD_DIR):
	mkdir -p -- "$(BUILD_DIR)"

$(BUILD_DIR)/eeprom_settings.txt: Makefile | $(BUILD_DIR)
	$(info EEPROM settings:)
	$(info $(HAT_EEPROM_SETTINGS))
	$(file > $@,$(HAT_EEPROM_SETTINGS))

$(BUILD_DIR)/sx1255_raspberrypi.dtbo: sx1255_raspberrypi.dts | $(BUILD_DIR)
	dtc -Wno-unit_address_vs_reg "$<" > "$@"

$(BUILD_DIR)/eeprom.bin: $(BUILD_DIR)/eeprom_settings.txt $(BUILD_DIR)/sx1255_raspberrypi.dtbo | $(BUILD_DIR)
	"$(EEPMAKE)" "$<" "$@" "$(BUILD_DIR)/sx1255_raspberrypi.dtbo"

write_eeprom: $(BUILD_DIR)/eeprom.bin
	"$(EEPFLASH)" -w "-f=$<" -t=24c256

clean:
	rm -rf -- "$(BUILD_DIR)"

.PHONY: all write_eeprom clean
