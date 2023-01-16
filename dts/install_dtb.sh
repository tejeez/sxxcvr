#!/bin/sh
dtc sx1255.dts | sudo tee /boot/overlays/sx1255.dtbo >/dev/null
