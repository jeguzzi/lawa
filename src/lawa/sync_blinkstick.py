from __future__ import division
import time
from blinkstick.blinkstick import (BlinkStick, _find_blicksticks, BlinkStickException,
                                   _remap_rgb_value_reverse)
from random import randint
from usb.core import USBError
from threading import RLock


def find_first():
    d = _find_blicksticks(find_all=False)
    if d:
        return SyncBlinkStick(device=d)


def _rgb2dict(r, g, b):
    return {'red': r, 'green': g, 'blue': b}


class SyncBlinkStick(BlinkStick):
    """ Allows to set colors (and pulse, morph, blink) multiple leds at the same time
        Extend API of set_color, pulse, morph, and blink to accept a list of indices.
        and a dictionary as a target
        >> bs.set_color(index=[1,2,3], name='pink')
        >> bs.set_color(index={1: {'name':'pink'}, 2: {'red': 100)}})
        """

    def __init__(self, device=None, error_reporting=True):
        super(SyncBlinkStick, self).__init__(device=device, error_reporting=error_reporting)
        self.max_value_single_led = 255.0
        self.lock = RLock()

    def set_color(self, channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None):
        self.lock.acquire(True)
        #     print('Could not acquire lock')
        #     return
        # else:
        #     print('locked')
        if type(index) == dict:
            for i, color in index.items():
                try:
                    super(SyncBlinkStick, self).set_color(channel=channel, index=i, **color)
                except (BlinkStickException, USBError, AttributeError) as e:
                    print(e)
                    pass

        elif type(index) == list or type(index) == range:
            for i in index:
                try:
                    super(SyncBlinkStick, self).set_color(
                        channel=channel, index=i, red=red, green=green, blue=blue, name=name,
                        hex=hex)
                except (BlinkStickException, USBError, AttributeError) as e:
                    print(e)
                    pass
        else:
            try:
                super(SyncBlinkStick, self).set_color(
                    channel=channel, index=index, red=red, green=green, blue=blue, name=name,
                    hex=hex)
            except (BlinkStickException, USBError, AttributeError) as e:
                print(e)
                pass
        self.lock.release()

    def turn_off(self, index=[0]):
        if type(index) == int:
            index = [index]
        self.set_color(name='black', index=list(index))

    def morph(self, channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, duration=1000,
              steps=50):
        try:
            if type(index) == dict:
                gradient = {i: self._gradient(steps, i, **color) for i, color in index.items()}
            elif type(index) == list:
                gradient = {i: self._gradient(steps, i, red=red, green=green, blue=blue, name=name,
                                              hex=hex)
                            for i in index}
            else:
                gradient = {index: self._gradient(steps, index, red=red, green=green, blue=blue,
                                                  name=name, hex=hex)}
        except (BlinkStickException, USBError) as e:
            print(e)
            return
        self._morph_with_gradient(gradient, steps, duration=duration)

    def _determine_rgb_raw(self, red=0, green=0, blue=0, name=None, hex=None):
        try:
            if name:
                # Special case for name="random"
                if name == "random":
                    red = randint(0, 255)
                    green = randint(0, 255)
                    blue = randint(0, 255)
                else:
                    red, green, blue = self._name_to_rgb(name)
            elif hex:
                red, green, blue = self._hex_to_rgb(hex)
        except ValueError:
            red = green = blue = 0

        return red, green, blue

    def pulse(self, channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, repeats=1,
              duration=1000, steps=50):
        if type(index) == int:
            index = [index]
        self.turn_off(index=index)
        r, g, b = self._determine_rgb_raw(red=red, green=green, blue=blue, name=name, hex=hex)
        for x in range(repeats):
            self.morph(channel=channel, index=index, red=r, green=g, blue=b, duration=duration,
                       steps=steps)
            self.morph(channel=channel, index=list(index), red=0, green=0, blue=0,
                       duration=duration, steps=steps)

    def blink(self, channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None, repeats=1,
              delay=500):
        r, g, b = self._determine_rgb_raw(red=red, green=green, blue=blue, name=name, hex=hex)
        ms_delay = float(delay) / float(1000)
        for x in range(repeats):
            if x:
                time.sleep(ms_delay)
            self.set_color(channel=channel, index=index, red=r, green=g, blue=b)
            time.sleep(ms_delay)
            self.turn_off(index)

    def pulse_back(self, channel=0, index=0, red=0, green=0, blue=0, name=None, hex=None,
                   repeats=1, duration=1000, steps=50):
        if type(index) == int:
            index = [index]
        try:
            self.lock.acquire(True)
            rgb = {i: _rgb2dict(
                *_remap_rgb_value_reverse(
                    self._get_color_rgb(i), self.max_rgb_value)) for i in index}
            self.lock.release()
        except (BlinkStickException, USBError) as e:
            print(e)
            return
        for x in range(repeats):
            self.morph(channel=channel, index=index, red=red, green=green, blue=blue,
                       name=name, hex=hex, duration=duration, steps=steps)
            self.morph(channel=channel, index=rgb, duration=duration, steps=steps)
            # self.set_color(channel=channel, index=rgb)

    def _gradient(self, steps, index, red=0, green=0, blue=0, name=None, hex=None):
        r_end, g_end, b_end = self._determine_rgb_raw(red=red, green=green, blue=blue, name=name,
                                                      hex=hex)
        self.lock.acquire(True)
        r_start, g_start, b_start = _remap_rgb_value_reverse(
            self._get_color_rgb(index), self.max_rgb_value)
        self.lock.release()
        if r_start > 255 or g_start > 255 or b_start > 255:
            r_start = 0
            g_start = 0
            b_start = 0
        gradient = []
        for n in range(0, steps + 1):
            d = 1.0 * n / steps
            r = (r_start * (1 - d)) + (r_end * d)
            g = (g_start * (1 - d)) + (g_end * d)
            b = (b_start * (1 - d)) + (b_end * d)
            gradient.append({'red': r, 'green': g, 'blue': b})
        return gradient

    def _morph_with_gradient(self, gradient, steps=50, duration=1000):
        ms_delay = float(duration) / float(1000 * steps)
        colors = [{i: gradient[i][n] for i in gradient} for n in range(steps + 1)]
        for color in colors:
            self.set_color(index=color)
            time.sleep(ms_delay)
