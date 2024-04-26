#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Linear repeater example, demonstrating low latency full duplex operation.
"""

import logging
import os

import SoapySDR
import numpy as np
from scipy import signal

SAMPLERATE=75000.0

################################
####  Using the SDR device  ####
################################

def init_sdr():
    """Configure the SDR and its RX and TX streams."""
    dev = SoapySDR.Device({
        'driver': 'sx',
    })
    dev.setSampleRate(SoapySDR.SOAPY_SDR_RX, 0, SAMPLERATE)
    dev.setSampleRate(SoapySDR.SOAPY_SDR_TX, 0, SAMPLERATE)

    dev.setFrequency(SoapySDR.SOAPY_SDR_RX, 0, 432.55e6)
    dev.setFrequency(SoapySDR.SOAPY_SDR_TX, 0, 434.55e6)

    dev.setGain(SoapySDR.SOAPY_SDR_RX, 0, 50.0)
    dev.setGain(SoapySDR.SOAPY_SDR_TX, 0, 0.0)

    # Set transmitter enable threshold to 0 to keep transmitter always on.
    rx = dev.setupStream(SoapySDR.SOAPY_SDR_RX, SoapySDR.SOAPY_SDR_CF32, [0], {})
    tx = dev.setupStream(SoapySDR.SOAPY_SDR_TX, SoapySDR.SOAPY_SDR_CF32, [0], {'threshold':'0'})
    return (dev, rx, tx)

class FullDuplexIo:
    """Full duplex signal I/O using SoapySDR."""
    def __init__(self, device, rx_stream, tx_stream, buffer_samples = 256, latency_samples = 256*3):
        self.buf = np.zeros(buffer_samples, dtype=np.complex64)
        #self.tx_start_buf = np.zeros(latency_samples, dtype=np.complex64)
        # Convert latency from samples to nanoseconds
        self.rx_tx_time_diff = int(round(latency_samples * 1e9 / SAMPLERATE))
        self.dev = device
        self.rx = rx_stream
        self.tx = tx_stream
        self.dev.activateStream(self.rx)
        self.dev.activateStream(self.tx)

    def run(self, process):
        """Call this in a loop as long as the application should be running.

        "process" is a callback that should process the received signal
        in the given buffer and write transmit signal into the same buffer.
        The function should not change the length of the buffer.
        """
        rxret = self.dev.readStream(self.rx, [self.buf], len(self.buf))
        logging.info('RX ret %s' % rxret)
        if rxret.ret != len(self.buf):
            logging.warning('RX read failed: %s' % rxret)
            return

        process(self.buf)
        txret = self.dev.writeStream(
            self.tx,
            [self.buf], len(self.buf),
            flags = SoapySDR.SOAPY_SDR_HAS_TIME,
            timeNs = rxret.timeNs + self.rx_tx_time_diff
        )
        if txret.ret != len(self.buf):
            logging.warning('TX write failed: %s' % txret)


#############################
####  Signal processing  ####
#############################

class IirFilter:
    """Wrap scipy lfilter and related functions to make them nicer to use."""
    def __init__(self, coefficients):
        self.b, self.a = coefficients[0], coefficients[1]
        self.z = signal.lfiltic(self.b, self.a, np.zeros(1, dtype=np.complex64))

    def process(self, s):
        filtered, self.z = signal.lfilter(self.b, self.a, s, zi=self.z)
        return filtered

def clip_signal(s):
    """Limit to a maximum magnitude of 1."""
    s /= np.maximum(np.abs(s), 1.0)

class LinearRepeaterDsp:
    def __init__(self, fs=SAMPLERATE):
        self.dc_blocker = IirFilter(signal.butter(1, 100.0, btype='highpass', output='ba', fs=fs))
        self.channel_filter1 = IirFilter(signal.butter(4, 12000.0, btype='lowpass', output='ba', fs=fs))
        self.channel_filter2 = IirFilter(signal.butter(4, 12000.0, btype='lowpass', output='ba', fs=fs))

    def process(self, buf):
        s = self.dc_blocker.process(buf)
        s = self.channel_filter1.process(s)
        # To make it a bit simpler, no AGC is implemented for now.
        # Just amplify and clip samples with a too high amplitude.
        s *= 1000.0
        clip_signal(s)
        s *= 0.3
        # Filter the signal again after clipping
        # to avoid splatter to other channels.
        s = self.channel_filter2.process(s)
        buf[:] = s


#########################
####  Main function  ####
#########################

def main():
    logging.basicConfig(format='%(asctime)s %(levelname)-8s %(message)s')
    #SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_DEBUG)

    try:
        os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(10))
    except PermissionError:
        logging.warning("Could not use real-time scheduling. For best results, add the following line in /etc/security/limits.conf: %s - rtprio 50" % (os.getlogin(),))

    dsp = LinearRepeaterDsp()
    device, rx_stream, tx_stream = init_sdr()
    io = FullDuplexIo(device, rx_stream, tx_stream)
    while True:
        io.run(dsp.process)

if __name__ == '__main__':
    main()
