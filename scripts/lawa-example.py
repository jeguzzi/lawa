#!/usr/bin/env python

import lawa
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from mavros_msgs.msg import State

@lawa.add('mavros/battery', BatteryState)
def battery(msg, bs):
    p = msg.percentage
    red = 255 * (1 - p)
    green = 255 * p
    if p > 0.2:
        bs.set_color(red=red, green=green, blue=0, index=0)
    else:
        bs.set_color(name='red', index=0)


@lawa.add('localization', String)
def localization(msg, bs):
    color = {'gps': 'blue', 'optitrack': 'yellow', 'pozyx': 'green'}.get(msg.data, 'black')
    bs.morph(name=color, index=1, duration=500)


lawa.pulse('pozyx/pose', name='green', index=2, sample=1)
lawa.pulse('/optitrack/{ns}', name='yellow', index=3, sample=5)
lawa.pulse('gps/fix', name='blue', index=4, sample=5)
lawa.pulse('mavros/setpoint_raw/global', name='red', index=5, sample=1)

if __name__ == '__main__':
    lawa.run()
