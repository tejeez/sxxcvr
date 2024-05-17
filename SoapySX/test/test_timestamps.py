#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Simple test for timestamped transmit.

Transmit a short burst of signal at given intervals.
If it works, the transmit LED should flash at that interval."""

import time

import SoapySDR
import numpy as np

def main(tx_interval = 1000_000_000, tx_latency = 10_000_000):
    SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_DEBUG)

    device = SoapySDR.Device({
        'driver': 'sx',
    })

    device.setSampleRate(SoapySDR.SOAPY_SDR_RX, 0, 75000.0)
    device.setSampleRate(SoapySDR.SOAPY_SDR_TX, 0, 75000.0)

    device.setFrequency(SoapySDR.SOAPY_SDR_RX, 0, 433.9e6)
    device.setFrequency(SoapySDR.SOAPY_SDR_TX, 0, 433.92e6)

    rx = device.setupStream(SoapySDR.SOAPY_SDR_RX, SoapySDR.SOAPY_SDR_CF32, [0], {})
    tx = device.setupStream(SoapySDR.SOAPY_SDR_TX, SoapySDR.SOAPY_SDR_CF32, [0], {})

    device.activateStream(rx)
    device.activateStream(tx)

    rxbuf = np.zeros(256, dtype=np.complex64)
    txbuf = np.ones(256, dtype=np.complex64)
    t_prev = 0
    while True:
        r = device.readStream(rx, [rxbuf], len(rxbuf))
        t = r.timeNs // tx_interval
        if t > t_prev:
            r = device.writeStream(tx, [txbuf], len(txbuf), SoapySDR.SOAPY_SDR_HAS_TIME, r.timeNs + tx_latency)
        t_prev = t

    device.deactivateStream(rx)
    device.deactivateStream(tx)

if __name__ == '__main__':
    main()
