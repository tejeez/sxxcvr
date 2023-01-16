#!/usr/bin/env python3
"""Some tests for the SoapySX module."""

import SoapySDR

def main():
    device = SoapySDR.Device({
        'driver': 'sx',
    })

if __name__ == '__main__':
    main()
