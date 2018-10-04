import rospy
from rospy.msg import AnyMsg
from .blinkstick_driver import blinkstickROS


# The following interfaces are available to set colors:
# pulse(channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, repeats=1, duration=1000,
#       steps=50)
# blink(channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, repeats=1, delay=500)
# morph(channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, duration=1000, steps=50)
# set_color(channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None)

_lawa = []


def _valid_time(last_msg, period):
    t = rospy.Time.now()
    if last_msg['stamp'] is None or (t - last_msg['stamp']).to_sec() > period:
        last_msg['stamp'] = t
        return True
    return False


def _valid_seq(last_msg, sample):
    last_msg['seq'] += 1
    return last_msg['seq'] % sample == 0


def _pulse(f):
    def h(topic_name, duration=100, steps=10, sample=1, period=1, repeats=1, **kwargs):
        last_msg = {'stamp': None, 'seq': 0}

        def g(msg, bs):
            if _valid_seq(last_msg, sample) and _valid_time(last_msg, period):
                f(bs, duration=duration, steps=steps, **kwargs)
        _lawa.append((topic_name, AnyMsg, g))
        return g
    return h


@_pulse
def pulse(bs, duration=100, steps=10, **kwargs):
    bs.pulse(duration=duration, steps=steps, **kwargs)


@_pulse
def pulse_to(bs, duration=100, steps=20, **kwargs):
    index = kwargs.get('index', 0)
    bs.morph(duration=duration, steps=steps, index=index, name='black')
    bs.morph(duration=duration, steps=steps, **kwargs)


@_pulse
def pulse_back(bs, duration=100, steps=10, **kwargs):
    bs.pulse_back(duration=duration, steps=steps, **kwargs)


def add(topic_name, msg_type, period=1.0):
    last_msg = {'stamp': None}

    def dec(f):
        def g(msg, bs):
            t = rospy.Time.now()
            if last_msg['stamp'] is None or (t - last_msg['stamp']).to_sec() > period:
                last_msg['stamp'] = t
                rospy.loginfo(f.__name__)
                f(msg, bs)

        _lawa.append((topic_name, msg_type, g))
    return dec


def run(init=None):
    bs = blinkstickROS()
    if init:
        init(bs)
    params = rospy.get_param('~', {})
    for topic_name, msg_type, f in _lawa:
        rospy.Subscriber(topic_name.format(**params), msg_type, f, callback_args=bs, queue_size=2)
    rospy.spin()
    bs.turn_off(list(range(8)))
