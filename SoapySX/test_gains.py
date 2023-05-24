#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
'''Some tests for the SoapySX module.'''

import SoapySDR

def main():
    SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_DEBUG)

    device = SoapySDR.Device({
        'driver': 'sx',
    })

    SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_INFO)
    print('RX gain range:', device.getGainRange(SoapySDR.SOAPY_SDR_RX, 0))
    print('Automatically distributed RX gains:')
    for gain in range(-10,80):
        device.setGain(SoapySDR.SOAPY_SDR_RX, 0, gain)
        gain_lna   = device.getGain(SoapySDR.SOAPY_SDR_RX, 0, 'LNA')
        gain_pga   = device.getGain(SoapySDR.SOAPY_SDR_RX, 0, 'PGA')
        gain_total = device.getGain(SoapySDR.SOAPY_SDR_RX, 0)
        print('%5.1f -> %5.1f + %5.1f = %5.1f' % (gain, gain_lna, gain_pga, gain_total))

    print('TX gain range:', device.getGainRange(SoapySDR.SOAPY_SDR_TX, 0))
    print('Automatically distributed TX gains:')
    for gain in range(-40,10):
        device.setGain(SoapySDR.SOAPY_SDR_TX, 0, gain)
        gain_dac   = device.getGain(SoapySDR.SOAPY_SDR_TX, 0, 'DAC')
        gain_mixer = device.getGain(SoapySDR.SOAPY_SDR_TX, 0, 'MIXER')
        gain_total = device.getGain(SoapySDR.SOAPY_SDR_TX, 0)
        print('%5.1f -> %5.1f + %5.1f = %5.1f' % (gain, gain_dac, gain_mixer, gain_total))

    SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_DEBUG)


if __name__ == '__main__':
    main()
