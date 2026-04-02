#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import threading

import SoapySDR
import numpy as np

class Measurement:
    def __init__(
        self,
        # How many samples to use for RX measurement
        rx_measurement_length = 8192,
        # RX intermediate frequency as a number of cycles within RX measurement.
        # The frequency should be aligned on a tuning step.
        # This can be achieved by making both rx_measurement_length
        # and rx_if_cycles_in_measurement powers of two.
        rx_if_cycles_in_measurement = 128,
        # How long to wait for PLLs to settle after changing frequencies
        pll_lock_margin_ns = 5000000,
        dac_value = 1.0+1.0j
    ):
        self.running = True
        self.tx_ready = False
        self._tx_thread = None
        self.pll_lock_margin_ns = pll_lock_margin_ns

        self.dev = SoapySDR.Device({'driver': 'sx'})

        self._sample_rate = 300000.0
        self.dev.setSampleRate(SoapySDR.SOAPY_SDR_RX, 0, self._sample_rate)
        self.dev.setSampleRate(SoapySDR.SOAPY_SDR_TX, 0, self._sample_rate)

        self.dev.setGain(SoapySDR.SOAPY_SDR_RX, 0, 'LNA', 24.0)
        self.dev.setGain(SoapySDR.SOAPY_SDR_RX, 0, 'PGA', 16.0)
        self.dev.setGain(SoapySDR.SOAPY_SDR_TX, 0, 'DAC', 6.0)
        self.dev.setGain(SoapySDR.SOAPY_SDR_TX, 0, 'MIXER', 30.0)

        # How many samples to read at a time to wait to receive the right signal
        rx_wait_length = rx_measurement_length // 8

        # RX intermediate frequency in Hz
        self._rx_if = self._sample_rate * rx_if_cycles_in_measurement / rx_measurement_length

        # Create a signal to correlate with received signal
        window = np.hanning(rx_measurement_length)
        self._tone = np.exp(np.linspace(
            0.0,
            -2.0j * np.pi * rx_if_cycles_in_measurement,
            rx_measurement_length,
            endpoint=False,
            dtype=np.complex64
        )) * window * (1.0 / np.sum(window))

        self._tx_signal = np.full(rx_wait_length, dac_value, dtype=np.complex64)
        self._rx_buffer = np.zeros(rx_measurement_length, dtype=np.complex64)
        self._rx_wait_buffer = np.zeros(rx_wait_length, dtype=np.complex64)

        self.rx = self.dev.setupStream(SoapySDR.SOAPY_SDR_RX, SoapySDR.SOAPY_SDR_CF32, [0], {'period': str(rx_wait_length)})
        self.tx = self.dev.setupStream(SoapySDR.SOAPY_SDR_TX, SoapySDR.SOAPY_SDR_CF32, [0], {'period': str(rx_wait_length)})

        self.dev.activateStream(self.rx)
        self.dev.activateStream(self.tx)

        self._tx_thread = threading.Thread(target=self._tx_thread_main)
        self._tx_thread.start()

    def stop(self):
        self.running = False
        if self._tx_thread is not None:
            self._tx_thread.join()

    def _tx_thread_main(self):
        while self.running:
            self.dev.writeStream(self.tx, [self._tx_signal], len(self._tx_signal))
            self.tx_ready = True
        self.tx_ready = False

    def measure(self, frequency):
        self.dev.setFrequency(SoapySDR.SOAPY_SDR_RX, 0, frequency - self._rx_if)
        self.dev.setFrequency(SoapySDR.SOAPY_SDR_TX, 0, frequency)
        # New frequency has been written to the chip when setFrequency returns.
        # Check the hardware time now.
        # Make short reads of RX stream until we get a signal after this point.
        # Add some margin to have some time for PLLs to lock.
        frequency_changed_time = self.dev.getHardwareTime()

        # Make sure something is being transmitted first
        while self.running and not self.tx_ready:
            self.dev.readStream(self.rx, [self._rx_wait_buffer], len(self._rx_wait_buffer))

        while self.running:
            ret = self.dev.readStream(self.rx, [self._rx_wait_buffer], len(self._rx_wait_buffer))
            if ret.ret < 0:
                print('RX wait error:', ret)
                return
            next_rx_time = ret.timeNs + SoapySDR.ticksToTimeNs(ret.ret, self._sample_rate)
            if next_rx_time - frequency_changed_time >= self.pll_lock_margin_ns:
                break

        ret = self.dev.readStream(self.rx, [self._rx_buffer], len(self._rx_buffer))
        if ret.ret != len(self._tone):
            print('RX error:', ret)
            return

        correlation = np.dot(self._rx_buffer, self._tone)
        return 10.0 * np.log10(correlation.real ** 2 + correlation.imag ** 2)



def main(freq_start = 432.1e6, freq_step = 0.2e6, freq_num = 30):
    measurement = Measurement()

    for freq_i in range(freq_num):
        freq = freq_start + freq_step * freq_i
        db = measurement.measure(freq)
        bar = int(round(min(max((db + 120.0) / 2.0, 0), 55)))
        print('%8.2f MHz %7.2f dB %s' % (freq * 1e-6, db, '#'*bar))

    measurement.stop()


if __name__ == '__main__':
    main()
