import rospy
from blinkstick_driver import blinkstickROS
from rospy.msg import AnyMsg

# pulse(channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, repeats=1, duration=1000,
#       steps=50)
# blink(channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, repeats=1, delay=500)
# morph(channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, duration=1000, steps=50)
# set_color(channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None)

_lawa = []


def pulse(topic_name, duration=100, steps=10, **kwargs):
    def f(msg, bs):
        bs.pulse(duration=duration, steps=steps, **kwargs)
    _lawa.append((topic_name, AnyMsg, f))


def add(topic_name, msg_type):
    def dec(f):
        _lawa.append((topic_name, msg_type, f))
    return dec


def run():
    bs = blinkstickROS()
    params = rospy.get_param('~', {})
    for topic_name, msg_type, f in _lawa:
        rospy.Subscriber(topic_name.format(**params), msg_type, f, callback_args=bs)
    rospy.spin()
