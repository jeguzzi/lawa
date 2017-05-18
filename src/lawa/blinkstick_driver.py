#!/usr/bin/env python
from __future__ import print_function
import rospy
import blinkstick.blinkstick as bs
from std_msgs.msg import ColorRGBA


class blinkstickROS(object):

    def __getattr__(self, attr):
        a = getattr(self.bs, attr)
        if callable(a):
            def f(*args, **kwargs):
                if 'indexes' in kwargs:
                    indexes = kwargs.pop('indexes')
                    for i in indexes:
                        a(*args, index=i, **kwargs)
                else:
                    a(*args, **kwargs)
            return f
        return a

    def __init__(self):
        rospy.init_node('blinkstick', anonymous=True)
        self.bs = bs.find_first()
        if not self.bs:
            rospy.logerr('No blinkstick device found')
            exit(1)

        number_of_leds = rospy.get_param('~leds', 1)
        max_intensity = rospy.get_param('~max_intensity', 255)
        self.set_max_rgb_value(max_intensity)
        for i in range(number_of_leds):
            rospy.Subscriber('led_{0}'.format(
                i), ColorRGBA, self.set_color_from_msg, callback_args=i)

    def set_color_from_msg(self, msg, i):
        rospy.loginfo('%s %d', msg, i)
        self.set_color(channel=0, index=i, red=(255 * msg.r), green=(255 * msg.g),
                       blue=(255 * msg.b))


if __name__ == '__main__':
    blinkstickROS()
    rospy.spin()
