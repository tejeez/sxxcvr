## Setup on Raspberry Pi

Install dependencies:
```
sudo apt-get install --no-install-recommends git device-tree-compiler alsa-utils make g++ cmake libsoapysdr-dev libasound2-dev libgpiod-dev soapysdr-tools python3-soapysdr
```

Install device tree blob:
```
cd dts
./install_raspberrypi.sh
```

Reboot the machine and check that the audio device is found:
```
aplay -L|grep SX1255
arecord -L|grep SX1255
```

Compile and install SoapySDR module:
```
cd SoapySX
mkdir build
cd build
cmake ..
make -j4
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
script for a simple example on using this feature.
