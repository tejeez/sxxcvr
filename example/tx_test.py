#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Simple transmit test program.

Parameters can be changed by importing the module in Python REPL
and calling main() with different parameters.
For example:
$ python3
>>> import tx_test
>>> tx_test.main(tx_freq = 435e6)
"""

import logging
import os
import time

import SoapySDR
import numpy as np

def main(
    tx_freq = 433.9e6,
    dac_value = 1.0+1.0j,
    tx_gain = 10.0,
    blocksize = 4096,
    loglevel = logging.WARNING,
    soapyloglevel = SoapySDR.SOAPY_SDR_INFO
):
    logging.basicConfig(format='%(asctime)s %(levelname)-8s %(message)s', level=loglevel)
    SoapySDR.setLogLevel(soapyloglevel)

    try:
        os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(10))
    except PermissionError:
        logging.warning('Could not use real-time scheduling')

    buf = np.full(blocksize, dac_value, dtype=np.complex64)

    dev = SoapySDR.Device({
        'driver': 'sx',
    })
    dev.setSampleRate(SoapySDR.SOAPY_SDR_TX, 0, 75000.0)
    dev.setFrequency(SoapySDR.SOAPY_SDR_TX, 0, tx_freq)
    dev.setGain(SoapySDR.SOAPY_SDR_TX, 0, tx_gain)

    tx = dev.setupStream(SoapySDR.SOAPY_SDR_TX, SoapySDR.SOAPY_SDR_CF32, [0], {'threshold':'0'})
    dev.activateStream(tx)
    while True:
        t1 = time.time()
        ret = dev.writeStream(tx, [buf], len(buf))
        t2 = time.time()
        logging.info('%.4f' % (t2-t1))
        if ret.ret != len(buf):
            logging.warning('TX: %s' % ret)


if __name__ == '__main__':
    main()
