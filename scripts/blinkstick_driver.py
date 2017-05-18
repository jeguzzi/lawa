#!/usr/bin/env python
import rospy
from lawa.blinkstick_driver import blinkstickROS


if __name__ == '__main__':
    blinkstickROS()
    rospy.spin()
