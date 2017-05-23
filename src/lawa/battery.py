from __future__ import division


def _rgb_line(percent, i, num_of_leds):
    i = num_of_leds - i - 1
    j = (i + 1) / num_of_leds
    i = i / num_of_leds
    green = i * 255
    red = 255 - green
    p = percent * 0.01

    if p > j:
        v = 1
    elif p < i:
        v = 0
    else:
        v = (p - i) / (j - i)
    return {'red': v * red, 'green': v * green}


def single(percent, num_of_leds=1):
    green = percent * 255
    red = 255 - green
    return {'red': red, 'green': green}


def line(percent, number_of_leds=1):
    return {i: _rgb_line(percent, i, number_of_leds) for i in range(number_of_leds)}


# def battery(topic_name, msg_type, linear=False, num_of_leds=1, min_rate=1, max_rate=5,
#             min_percent_to_blink=10):
#
#     battery = {'percent': 100}
#     def g(evt):
#         percent = battery['percent']
#         if percent is None:
#             return
#         if percent < min_percent_to_blink:
#             bs.
#         if linear:
#             colors = _linear(percent, num_of_leds)
#
#     rospy.Timer(rospy.Duration(1), g)
#     def f(msg, bs):
#         try:
#             percent = msg.percent
#         except AttributeError:
#             try:
#                 percent = msg.percentage * 100
#             except AttributeError:
#                 percent = None
