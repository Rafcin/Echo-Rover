#!/usr/bin/env python
from __future__ import print_function
import unittest

# import b2_logic
from b2_logic.nodes.ir_sensors import (
    volts_per_adc, volts_at_cm_distance, adc_at_proximity_dist, MCP3008Stub
)

PKG = 'b2'
NAME = 'b2_ir_sensors_unittest'


class TestIRSensors(unittest.TestCase):

    def setUp(self):
        print()

    def test_volts_per_adc(self):
        # (vref, min_adc_reading, max_adc_reading, expected)
        tests = [
            (5.0, 0, 1023, round(5.0/1023, 3)),
            (3.3, 0, 1023, round(3.3/1023, 3))
        ]

        print()
        for vref, min_adc, max_adc, expected in tests:
            actual = round(
                        volts_per_adc(
                            vref,
                            min_adc,
                            max_adc), 3)
            print("Actual: {}, Expected: {}".format(actual, expected))
            self.assertEqual(actual, expected)

    def test_volts_at_dist_cm(self):
        # (cm, expected)
        tests = [
            (10, 3.649),
            (20, 2.016),
            (30, 1.413),
            (50, 0.978),
            (70, 0.822),
            (100, 0.724),
            (150, 0.663)
        ]

        print()
        for cm, expected in tests:
            actual = round(volts_at_cm_distance(cm), 3)
            print("Actual: {}, Expected: {}".format(actual, expected))
            self.assertEqual(actual, expected)

    def test_adc_at_prox_dist(self):
        # (dist_m, v_per_adc, expected)
        tests = [
            (0.1, 5.0/1023, 746),
            (0.5, 5.0/1023, 200),
            (1.0, 5.0/1023, 148),
            (1.5, 5.0/1023, 135),
            (3.3, 5.0/1023, 125),
            (4.7, 5.0/1023, 124),
            (5.0, 5.0/1023, 123),
            (0.1, 3.3/1023, 1131),
            (0.5, 3.3/1023, 303),
            (1.0, 3.3/1023, 224),
            (1.3, 3.3/1023, 210),
            (2.7, 3.3/1023, 192),
            (3.0, 3.3/1023, 191),
            (3.3, 3.3/1023, 190),
        ]

        print()
        for dist_m, v_per_adc, expected in tests:
            actual = adc_at_proximity_dist(dist_m, v_per_adc)
            print("Actual: {}, Expected: {}".format(actual, expected))
            self.assertEqual(actual, expected)

    def test_MCP3008stub(self):
        # (channel, val)
        tests = [val * 150 for val in range(8)]
        stub = MCP3008Stub()

        print()
        for idx, val in enumerate(tests):
            stub.set_adc(idx, val)
            actual = stub.read_adc(idx)
            expected = val
            print("Actual: {}, Expected: {}".format(actual, expected))
            self.assertEqual(actual, expected)

    def test_load_adafruit_mcp3008(self):
        import Adafruit_MCP3008
        import Adafruit_GPIO


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestIRSensors)
