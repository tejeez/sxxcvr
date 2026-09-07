## Setup on Raspberry Pi

Install dependencies:
```
sudo apt-get install --no-install-recommends git make g++ cmake libsoapysdr-dev libasound2-dev soapysdr-tools python3-soapysdr
```

Some prototype boards do not have the HAT identification EEPROM written.
If you have one of those, write it first by following
[EEPROM writing instructions](dts/README.md).

Compile and install SoapySDR module:
```
cd SoapySX
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

Check that the module is found:
```
SoapySDRUtil --probe=driver=sx
```

## M17 Project SX1255 HAT

The [M17 Project board](https://github.com/M17-Project/SX1255_HAT-hw) uses the
same SX1255 SPI and I2S interfaces but has a different reset pin and no
external RX/TX switch GPIOs. Select its profile with `board=m17`:

```
SoapySDRUtil --probe="driver=sx,board=m17"
```

The M17 HAT has no identification EEPROM, so its device-tree overlay must be
installed manually. On the Raspberry Pi, build and install the supplied
overlay:

```
sudo apt-get install --no-install-recommends device-tree-compiler
cd dts
make overlays
sudo install -m 0644 build/sx1255_m17_raspberrypi.dtbo /boot/firmware/overlays/
```

Add these lines to `/boot/firmware/config.txt` and reboot:

```
dtparam=spi=on
dtoverlay=sx1255_m17_raspberrypi
```

On older Raspberry Pi OS releases, use `/boot/overlays/` and
`/boot/config.txt` instead. When this overlay is loaded, SoapySX detects the
M17 profile automatically, so `SoapySDRUtil --probe=driver=sx` is sufficient.

If an existing I2S-slave overlay creates a differently named ALSA card, select
it with `alsa_card`:

```
SoapySDRUtil --probe="driver=sx,board=m17,alsa_card=0"
```

The M17 profile uses the board's fixed 32 MHz TCXO, BCM GPIO25 (header pin 22)
for reset, `/dev/spidev0.0`, and the `SX1255` ALSA card by default. The
`reset_gpio`, `spi`, `gpiochip`, `alsa_card`, `alsa_rx`, `alsa_tx`, and
`master_clock` arguments override those defaults when needed. `alsa_rx` and
`alsa_tx` are mainly intended for API callers; use `alsa_card` in a
comma-separated SoapySDR command-line device string.

## SX1255 gain stages

Applications can set the SX1255 analog stages by name through the SoapySDR
gain API:

| Direction | Stage | Range | Nominal step |
|---|---|---:|---:|
| RX | `LNA` | 0–48 dB | 6 dB |
| RX | `PGA` | 0–30 dB | 2 dB |
| TX | `DAC` | 0–9 dB | 3 dB |
| TX | `MIXER` | 0–30 dB | 2 dB |

For example, Python callers can request explicit RX placement with:

```python
device.setGain(SoapySDR.SOAPY_SDR_RX, 0, "LNA", 36)
device.setGain(SoapySDR.SOAPY_SDR_RX, 0, "PGA", 14)
```

Aggregate gain calls remain supported for compatibility. SoapySX distributes
an aggregate request between the applicable stages and quantizes each stage to
a value supported by the chip. Applications that need repeatable noise figure,
headroom, or TX linearity should use the named stages and read back the applied
values.

## Features
SoapySX provides some support for timestamps which are used by some
applications to obtain a known timing relationship between transmitted and
received signals.
See the
[linear repeater example](example/linear_repeater.py)
for an example on using timestamps to obtain a constant, known latency
from received to transmitted signal.
