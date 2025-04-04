#
# Copyright 2015-2016 Ettus Research LLC
# Copyright 2018 Ettus Research, a National Instruments Company
# Copyright 2019 Ettus Research, a National Instruments Brand
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""Run device tests for the B2xx series."""

# pylint: disable=wrong-import-position
# pylint: disable=unused-import
# flake8: noqa

from usrp_probe_test import UhdUsrpProbeTest
from python_api_test import UhdPythonApiTest
from python_rx_stability_test import UhdPythonRxStabilityTest
from benchmark_rate_test import UhdBenchmarkRateTest

UhdBenchmarkRateTest.tests = {
    "mimo": {
        "duration": 1,
        "direction": "tx,rx",
        "chan": "0,1",
        "rate": 1e6,
        "products": [
            "B210",
        ],
        "acceptable-underruns": 20,
        "acceptable-overruns": 20,
        "acceptable-D": 0,
        "acceptable-S": 0,
    },
    "siso_chan0_slow": {
        "duration": 1,
        "direction": "tx,rx",
        "chan": "0",
        "rate": 1e6,
        "acceptable-underruns": 20,
        "acceptable-overruns": 20,
        "acceptable-D": 0,
        "acceptable-S": 0,
    },
    #'siso_chan0_fast': {
    #'duration': 1,
    #'direction': 'tx,rx',
    #'chan': '0',
    #'rate': 40e6,
    #'acceptable-underruns': 500,
    # },
    "siso_chan1_slow": {
        "duration": 1,
        "direction": "tx,rx",
        "chan": "1",
        "rate": 1e6,
        "acceptable-underruns": 20,
        "acceptable-overruns": 20,
        "acceptable-D": 0,
        "acceptable-S": 0,
        "products": [
            "B210",
        ],
    },
    #'siso_chan1_fast': {
    #'duration': 1,
    #'direction': 'tx,rx',
    #'chan': '1',
    #'rate': 40e6,
    #'acceptable-underruns': 500,
    #'products': ['B210',],
    # },
}

from rx_samples_to_file_test import RxSamplesToFileTest

RxSamplesToFileTest.tests = {
    "default": {
        "duration": 1,
        "subdev": "A:A",
        "rate": 5e6,
        "products": [
            "B210",
            "B200",
        ],
    },
}

from tx_waveforms_test import UhdTxWaveformsTest

UhdTxWaveformsTest.tests = {
    "chan0": {
        "chan": "0",
    },
    "chan1": {
        "chan": "1",
        "products": [
            "B210",
        ],
    },
    "both_chans": {
        "chan": "0,1",
        "products": [
            "B210",
        ],
    },
}

from tx_bursts_test import UhdTxBurstsTest
from test_pps_test import UhdTestPpsTest
from gpio_test import GpioTest
from list_sensors_test import ListSensorsTest
