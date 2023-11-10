# Writing HAT identification EEPROM

If your HAT already has its EEPROM written and up to date, you do not
have to do anything here. If not, follow these instructions:

## Install eepromutils and other tools

```
sudo apt-get install --no-install-recommends git make gcc device-tree-compiler alsa-utils
cd
git clone "https://github.com/raspberrypi/hats.git"
cd hats/eepromutils
make
sudo make install
```

## Make and write EEPROM image

`cd` to this directory:
```
cd dts
make clean
```

If you have a prototype version 1.0, do:
```
make HAT_VERSION=0x0100
```

For version 1.1, do this instead:
```
make HAT_VERSION=0x0101
```

Disable EEPROM write protection by placing a jumper between WP pins
on the board. Then write the EEPROM:
```
sudo make write_eeprom
```

Reboot your Raspberry Pi:
```
sudo reboot
```

After reboot, you can check that HAT ID data and audio device are found:
```
ls -l /proc/device-tree/hat
aplay -L|grep SX1255
arecord -L|grep SX1255
```
