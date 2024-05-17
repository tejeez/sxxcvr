## Setup on Raspberry Pi

Install dependencies:
```
sudo apt-get install --no-install-recommends git make g++ cmake libsoapysdr-dev libasound2-dev libgpiod-dev soapysdr-tools python3-soapysdr
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

## Features
SoapySX provides some support for timestamps which are used by some
applications to obtain a known timing relationship between transmitted and
received signals.
See the
[linear repeater example](example/linear_repeater.py)
for an example on using timestamps to obtain a constant, known latency
from received to transmitted signal.
