#!/usr/bin/env python3

PLOT=True

if PLOT:
    import matplotlib.pyplot as plt
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX, SOAPY_SDR_CF32

class Calibrator:
    def __init__(self):
        self.dev = SoapySDR.Device({'driver': 'sx'})
        self.fs = 150000.0
        if self.fs not in self.dev.listSampleRates(SOAPY_SDR_RX, 0):
            self.fs = 125000.0
        print('Using sample rate', self.fs)

        self.fftsize = 4096
        self.binfreq = self.fs / self.fftsize
        # Offset of RX and TX local oscillators as a multiple of bin size
        self.tx_rx_bins = 576
        # Offset of transmitted tone from TX LO
        # FIXME: Seems like it's on the wrong frequency. Testing with DC for now
        self.tx_tone_bins = 128
        # Amplitude of transmitted tone
        self.tx_tone_amplitude = 0.5

        self.fft_window = np.hanning(self.fftsize)
        # Find out correct FFT scaling to get actual DC levels and tone amplitudes
        self.fft_scaling = 1.0 / np.fft.fft(self.fft_window)[0]

        self.dev.setSampleRate(SOAPY_SDR_RX, 0, self.fs)
        self.dev.setSampleRate(SOAPY_SDR_TX, 0, self.fs)
        self.dev.setAntenna(SOAPY_SDR_RX, 0, 'LB')
        self.dev.setGain(SOAPY_SDR_RX, 0, 'LNA', 0)
        self.dev.setGain(SOAPY_SDR_RX, 0, 'PGA', 20)
        self.dev.setGain(SOAPY_SDR_TX, 0, 100)
        self.tune()
        self.rx = self.dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0], {'link':'1'})
        self.tx = self.dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0], {'link':'1'})

    def tune(self, freq = 435.0e6):
        self.dev.setFrequency(SOAPY_SDR_RX, 0, freq - self.binfreq * self.tx_rx_bins)
        self.dev.setFrequency(SOAPY_SDR_TX, 0, freq)

    def tx_rx(self, txsignal, rxlength):
        """Transmit a given signal and receive at the same time.
        Return the received signal."""
        rxbuf = np.zeros(rxlength, dtype=np.complex64)
        self.dev.activateStream(self.rx)
        self.dev.activateStream(self.tx)
        tx_ret = self.dev.writeStream(self.tx, [txsignal], len(txsignal))
        rx_ret = self.dev.readStream (self.rx, [rxbuf], len(rxbuf))
        self.dev.deactivateStream(self.rx)
        self.dev.deactivateStream(self.tx)
        if tx_ret.ret != len(txsignal):
            raise IOError('TX write returned %d' % tx_ret.ret)
        if rx_ret.ret != len(rxbuf):
            raise IOError('RX read returned %d' % rx_ret.ret)
        return rxbuf

    def tx_rx_analyze(self):
        # Add some extra signal before and after the part to be analyzed
        extra_before = 512
        extra_after = 512

        tx_length = self.fftsize + extra_before + extra_after
        tx_signal = self.tx_tone_amplitude * np.exp(np.linspace(
            0,
            np.pi * 2.0j * tx_length * self.tx_tone_bins / self.fftsize,
            tx_length,
            endpoint=False,
            dtype=np.complex64))
        rx_signal = self.tx_rx(tx_signal, rxlength=tx_length - extra_after)
        f = self.fft_scaling * np.fft.fft(
            self.fft_window *
            rx_signal[extra_before : extra_before+self.fftsize])

        if PLOT:
            plt.plot(np.abs(f))
        return {
            'RX_DC':    f[ 0],
            'TX_DC':    f[ self.tx_rx_bins],
            'TX_TONE':  f[ self.tx_rx_bins + self.tx_tone_bins],
            'TX_IMAGE': f[ self.tx_rx_bins - self.tx_tone_bins],
            'RX_IMAGE': f[-self.tx_rx_bins - self.tx_tone_bins],
        }

def print_dict(d):
    'Print dict of results in an easier to read way'
    for k, v in d.items():
        print('%10s %+E %+Ej' % (k, v.real, v.imag))

def main():
    calibrator = Calibrator()
    print('Before calibration:')
    for i in range(4):
        a = calibrator.tx_rx_analyze()
        print_dict(a)
    if PLOT:
        plt.show()
    calibrator.dev.setDCOffset(SOAPY_SDR_RX, 0, -a['RX_DC'])
    calibrator.dev.setIQBalance(SOAPY_SDR_RX, 0, -a['RX_IMAGE'] / a['TX_TONE'])
    print('After calibration:')
    for i in range(4):
        a = calibrator.tx_rx_analyze()
        print_dict(a)

if __name__ == '__main__':
    main()
