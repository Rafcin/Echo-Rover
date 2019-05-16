from __future__ import print_function

import rospy

from b2.msg import Proximity


class IRSensors:
    def __init__(self, pub_rate, num_adc_channels, vref, min_adc_val,
                 max_adc_val, proximity_dist, center_pub, mcp_obj):
        self._pub_rate = pub_rate
        self._num_adc_channels = num_adc_channels

        # Calculate the ADC value when an object is in "proximity"
        v_per_adc = volts_per_adc(vref, min_adc_val, max_adc_val)
        self._acd_at_prox_dist = adc_at_proximity_dist(proximity_dist, v_per_adc)
        rospy.logdebug("v_per_adc: {}".format(v_per_adc))
        rospy.logdebug("acd_at_prox_dist: {}".format(self._acd_at_prox_dist))

        self._center_pub = center_pub
        self._mcp = mcp_obj

    def run(self):

        try:
            while not rospy.is_shutdown():

                msg = Proximity()

                for channel in range(self._num_adc_channels):
                    is_proximity = False  # No object detected yet

                    # Read sensor values
                    val = self._mcp.read_adc(channel)
                    if val >= self._acd_at_prox_dist:
                        is_proximity = True

                    # # Flip for debugging
                    # if self._test_mode:
                    #     self._mcp.set_adc(channel, self._mcp.read_adc(channel) * -1)

                    # Publish sensor messages
                    msg.sensors.append(is_proximity)

                self._center_pub.publish(msg)
                self._pub_rate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")


def volts_at_cm_distance(dist_cm):
    # This function is the result of fitting the Voltage/Distance curve points in the
    # Sharp GP2Y0A60SZXF data sheet https://www.pololu.com/file/0J812/gp2y0a60szxf_e.pdf
    # using the site http://mycurvefit.com
    # The function takes in distance in cm, and outputs the voltage of the IR sensor's output
    return 0.5955366 + 6.8125134 / (1 + (dist_cm / 8.798111) ** 1.624654)


def adc_at_proximity_dist(prox_dist_m, v_per_adc):
    prox_dist_cm = prox_dist_m * 100
    v_at_prox_dist = volts_at_cm_distance(prox_dist_cm)
    return int(v_at_prox_dist / v_per_adc)


def volts_per_adc(vref, min_adc_reading, max_adc_reading):
    return vref / float(max_adc_reading - min_adc_reading)


class MCP3008Stub:
    def __init__(self):
        self.channels = [0] * 8

    def read_adc(self, channel):
        val = self.channels[channel]
        # For testing, flip the value each time it is read
        self.channels[channel] = val * -1
        return val

    def set_adc(self, channel, val):
        self.channels[channel] = val
