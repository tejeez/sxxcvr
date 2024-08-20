#!/usr/bin/env python3

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
        self.tx_tone_bins = 128
        # Amplitude of transmitted tone
        self.tx_tone_amplitude = 0.7

        # Initial calibration values
        self.rx_dc = 0.0 + 0.0j
        self.rx_iq = 0.0 + 0.0j
        self.tx_dc = 0.0 + 0.0j
        self.tx_iq = 0.0 + 0.0j

        self.fft_window = np.hanning(self.fftsize)
        # Find out correct FFT scaling to get actual DC levels and tone amplitudes
        self.fft_scaling = 1.0 / np.fft.fft(self.fft_window)[0]

        self.dev.setSampleRate(SOAPY_SDR_RX, 0, self.fs)
        self.dev.setSampleRate(SOAPY_SDR_TX, 0, self.fs)
        self.dev.setAntenna(SOAPY_SDR_RX, 0, 'LB')
        self.dev.setAntenna(SOAPY_SDR_TX, 0, 'TX')
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

        self.dev.setDCOffset (SOAPY_SDR_RX, 0, self.rx_dc)
        self.dev.setIQBalance(SOAPY_SDR_RX, 0, self.rx_iq)
        self.dev.setDCOffset (SOAPY_SDR_TX, 0, self.tx_dc)
        self.dev.setIQBalance(SOAPY_SDR_TX, 0, self.tx_iq)

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
        # Approximate additional delay from TX to RX in samples
        tx_rx_delay = 4

        tx_length = self.fftsize + extra_before + extra_after
        phase_per_sample = np.pi * 2.0j * self.tx_tone_bins / self.fftsize
        # Make the phase start from 0 in the beginning of analysis window
        initial_phase = phase_per_sample * (-extra_before + tx_rx_delay)
        tx_signal = self.tx_tone_amplitude * np.exp(np.linspace(
            initial_phase,
            initial_phase + phase_per_sample * tx_length,
            tx_length,
            endpoint=False,
            dtype=np.complex64))
        rx_signal = self.tx_rx(tx_signal, rxlength=tx_length - extra_after)
        f = self.fft_scaling * np.fft.fft(
            self.fft_window *
            rx_signal[extra_before : extra_before+self.fftsize])

        return {
            'RX_DC':    f[ 0],
            'TX_DC':    f[ self.tx_rx_bins],
            'TX_TONE':  f[ self.tx_rx_bins + self.tx_tone_bins],
            'TX_IMAGE': f[ self.tx_rx_bins - self.tx_tone_bins],
            'RX_IMAGE': f[-self.tx_rx_bins - self.tx_tone_bins],
        }

    def adjust_correction(self, a):
        'Adjust correction values based on results of tx_rx_analyze'
        self.rx_dc -= a['RX_DC']
        self.rx_iq -= a['RX_IMAGE'] / a['TX_TONE']
        self.tx_dc -= self.tx_tone_amplitude * a['TX_DC'] / a['TX_TONE']
        self.tx_iq -= a['TX_IMAGE'] / a['TX_TONE']

def print_dict(d):
    'Print dict of results in an easier to read way'
    for k, v in d.items():
        print('%10s %E %6.1f°' % (k, np.abs(v), np.degrees(np.angle(v))))

def main(iterations = 6):
    calibrator = Calibrator()
    for i in range(iterations):
        print('\033[32mCorrection values:\033[0m')
        print_dict({
            'RX DC offset ': calibrator.rx_dc,
            'RX IQ balance': calibrator.rx_iq,
            'TX DC offset ': calibrator.tx_dc,
            'TX IQ balance': calibrator.tx_iq,
        })
        a = calibrator.tx_rx_analyze()
        print('\033[32mMeasured tones:\033[0m')
        print_dict(a)
        # Adjust but skip on last iteration to just show the final results
        if i != iterations - 1:
            calibrator.adjust_correction(a)

if __name__ == '__main__':
    main()
