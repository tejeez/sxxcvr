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

    # Test reading all registers, including possible undocumented ones
    # with addresses above 0x13.
    # Reading past 0x7F apparently starts writing registers,
    # so stop there.
    for i, v in enumerate(device.readRegisters('', 0, 0x80)):
        print('%02X=%02X' % (i,v), end='\t ')
    print('')

    # Test writing registers.
    # This should start transmitting at 433.92 MHz.
    device.writeRegisters('', 4, (0xD8, 0xF5, 0xC3))
    device.writeRegisters('', 0, (0x0F,))

if __name__ == '__main__':
    main()
