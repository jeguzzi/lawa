#!/usr/bin/env python

from lawa import lawa
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from mavros_msgs.msg import State


@lawa.add('mavros/battery', BatteryState)
def battery(msg, bs):
    p = msg.percentage
    red = 255 * (1 - p)
    green = 255 * p
    if p > 0.2:
        bs.set_color(red=red, green=green, blue=0, index=4)
    else:
        bs.pulse(name='red', index=4, repeats=10, duration=500)
        bs.set_color(name='red', index=0)


@lawa.add('localization', String)
def localization(msg, bs):
    color = {'gps': 'blue', 'optitrack': 'cyan', 'pozyx': 'green'}.get(msg.data, 'orange')
    bs.morph(name=color, index=[0, 1, 2], duration=500)


@lawa.add('mavros/state', State)
def state(msg, bs):
    if not msg.connected:
        color = 'black'
    elif not msg.armed:
        color = 'red'
    elif not msg.guided:
        color = 'coral'
    else:
        color = 'darkorange'
    bs.morph(name=color, index=[5, 6, 7], duration=100)


lawa.pulse_to('pozyx/pose', name='green', index=0, sample=1, period=0)
lawa.pulse_to('/optitrack/{ns}', name='cyan', index=1, sample=1, period=1)
lawa.pulse_to('gps/fix', name='blue', index=2, sample=1, period=1)
lawa.pulse_to('mavros/setpoint_raw/global', name='red', index=6, sample=1, period=0)


if __name__ == '__main__':
    lawa.run()
