#!/usr/bin/env python
from __future__ import print_function

import Adafruit_GPIO
import Adafruit_MCP3008
import rospy

from b2.msg import Proximity
from b2_logic.nodes.ir_sensors import IRSensors, MCP3008Stub

DEFAULT_NODE_NAME = "ir_sensors"
DEFAULT_PROXIMITY_TOPIC = "~proximity"
DEFAULT_TEST_MODE = True
DEFAULT_PUB_HZ = 1  # hertz

DEFAULT_VREF = 5.0               # volts
DEFAULT_MIN_ADC_VAL = 0
DEFAULT_MAX_ADC_VAL = 1023
DEFAULT_PROXIMITY_DIST = 0.30    # meters
DEFAULT_NUM_ADC_CHANNELS = 1

# Hardware SPI configuration:
SPI_PORT = 0
SPI_DEVICE = 0


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)

    test_mode = rospy.get_param("~test_mode", DEFAULT_TEST_MODE)
    pub_rate = rospy.Rate(int(rospy.get_param("~pub_hz", DEFAULT_PUB_HZ)))
    num_adc_channels = rospy.get_param("~num_adc_channels", DEFAULT_NUM_ADC_CHANNELS)

    # Defined constants for the Sharp GP2Y0A60SZxF IR sensor
    vref = rospy.get_param("~vref", DEFAULT_VREF)
    min_adc_val = rospy.get_param("~min_adc_val", DEFAULT_MIN_ADC_VAL)
    max_adc_val = rospy.get_param("~max_adc_val", DEFAULT_MAX_ADC_VAL)
    proximity_dist = rospy.get_param("~proximity_distance", DEFAULT_PROXIMITY_DIST)

    # Publishes
    pub = rospy.Publisher(
        rospy.get_param("~proximity_topic", DEFAULT_PROXIMITY_TOPIC),
        Proximity,
        queue_size=1
    )

    # MCP3008 hardware interface object
    if not test_mode:
        mcp = Adafruit_MCP3008.MCP3008(
            spi=Adafruit_GPIO.SPI.SpiDev(SPI_PORT, SPI_DEVICE)
        )
    else:
        mcp = MCP3008Stub()
        for channel in range(num_adc_channels):
            mcp.set_adc(channel, 700)

    node = IRSensors(pub_rate, num_adc_channels, vref, min_adc_val,
                     max_adc_val, proximity_dist, pub, mcp)
    node.run()
