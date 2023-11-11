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
SoapySX does not currently support timestamps which are used by some
applications to obtain a known timing relationship between transmitted and
received signals. SoapySX, however, provides an alternative method to
synchronize receive and transmit streams by "linking" them together.
When streams are linked, they start running at the same time.
This behavior closely reflects that of the underlying ALSA API and its
[snd_pcm_link feature](https://www.alsa-project.org/alsa-doc/alsa-lib/pcm.html#pcm_sync).
See the
[test_linked_streams](SoapySX/test_linked_streams.py)
script or the
[linear repeater example](example/linear_repeater.py)
for examples on using this feature.
