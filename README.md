## Setup on Raspberry Pi

Install dependencies:
```
sudo apt-get install --no-install-recommends git device-tree-compiler alsa-utils make g++ cmake libsoapysdr-dev libasound2-dev soapysdr-tools python3-soapysdr
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
