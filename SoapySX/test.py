#!/usr/bin/env python3
"""Some tests for the SoapySX module."""

import traceback

import SoapySDR

def main():
    SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_DEBUG)

    device = SoapySDR.Device({
        'driver': 'sx',
    })

    # Test reading chip version number
    # using the raw SPI transfer API.
    print("%04x" % device.transactSPI(0, 0x0700, 16))

    def print_registers():
        """Test reading all registers, including possible undocumented ones
        with addresses above 0x13.
        Reading past 0x7F apparently starts writing registers,
        so stop there."""
        for i, v in enumerate(device.readRegisters('', 0, 0x80)):
            print('%02X=%02X' % (i,v), end='\t ')
        print('')

    print_registers()

    # Test writing registers.
    # Enable RX and TX
    device.writeRegisters('', 0, (0x0F,))

    # Test frequency settings.
    device.setFrequency(SoapySDR.SOAPY_SDR_RX, 0, 434.0123456789e6)
    device.setFrequency(SoapySDR.SOAPY_SDR_TX, 0, 434.123456789e6)
    print("getFrequency:",
        device.getFrequency(SoapySDR.SOAPY_SDR_RX, 0),
        device.getFrequency(SoapySDR.SOAPY_SDR_TX, 0))


    print_registers()

    # Test writing too many registers.
    # This should result in an error.
    try:
        device.writeRegisters('', 0x7E, (0, 0, 0,))
    except:
        traceback.print_exc()

if __name__ == '__main__':
    main()
