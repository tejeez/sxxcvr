#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Simple test for the linked streams feature."""

import time

import SoapySDR
import numpy as np

def main():
    SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_DEBUG)

    device = SoapySDR.Device({
        'driver': 'sx',
    })

    device.setSampleRate(SoapySDR.SOAPY_SDR_RX, 0, 75000.0)
    device.setSampleRate(SoapySDR.SOAPY_SDR_TX, 0, 75000.0)

    device.setFrequency(SoapySDR.SOAPY_SDR_RX, 0, 433.9e6)
    device.setFrequency(SoapySDR.SOAPY_SDR_TX, 0, 433.92e6)

    # Stream argument link=1 links the streams together,
    # so that they start at the same time.
    rx = device.setupStream(SoapySDR.SOAPY_SDR_RX, SoapySDR.SOAPY_SDR_CF32, [0], {'link':'1'})
    tx = device.setupStream(SoapySDR.SOAPY_SDR_TX, SoapySDR.SOAPY_SDR_CF32, [0], {'link':'1'})

    device.activateStream(rx)
    device.activateStream(tx)

    buf = np.zeros(256, dtype=np.complex64)
    initial_buf = np.zeros(256*4, dtype=np.complex64)
    # Fill up the TX buffer first.
    # First write to the TX buffer should start both the TX and RX streams
    # when they are linked together.
    device.writeStream(tx, [initial_buf], len(initial_buf))
    # Then keep reading and writing the same number of samples
    for _ in range(40):
        t1 = time.time()
        r = device.readStream(rx, [buf], len(buf))
        t2 = time.time()
        print('T:', t2-t1)
        print('RX:', r)
        if r.ret < 0:
            break
        r = device.writeStream(tx, [buf], len(buf))
        print('TX:', r)
        if r.ret < 0:
            break

    device.deactivateStream(rx)
    device.deactivateStream(tx)

if __name__ == '__main__':
    main()
