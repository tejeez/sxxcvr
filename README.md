## Setup on Raspberry Pi

Install dependencies:
```
sudo apt install git device-tree-compiler alsa-utils make g++ cmake libsoapysdr-dev
```

Install device tree blob:
```
cd dts
./install_dtb.sh
```

Add the following lines to `/boot/config.txt`:
```
dtparam=i2s=on
dtparam=spi=on
dtoverlay=sx1255
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
```
