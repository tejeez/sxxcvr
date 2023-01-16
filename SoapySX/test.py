#!/usr/bin/env python3
"""Some tests for the SoapySX module."""

import SoapySDR

def main():
    SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_DEBUG)

    device = SoapySDR.Device({
        'driver': 'sx',
    })

    # Test reading chip version number
    # using the raw SPI transfer API.
    print("%04x" % device.transactSPI(0, 0x0700, 16))

if __name__ == '__main__':
    main()
