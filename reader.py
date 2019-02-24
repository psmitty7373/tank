#!/usr/bin/python3

import pigpio, sys, time

MIN_SHORT = 789
MAX_SHORT = 989
MIN_LONG = 1678
MAX_LONG = 1878
SHORT_SPACE = 0
SHORT_PULSE = 2
LONG_SPACE = 4
LONG_PULSE = 6

STATE_START1 = 0
STATE_MID1 = 1
STATE_MID0 = 2
STATE_START0 = 3

TRANS_TABLE = [0x01, 0x91, 0x9b, 0xfb]

ADDRESS_MASK = 0xf
ADDRESS_SHIFT = 8
COMMAND_MASK = 0xff
COMMAND_SHIFT = 0

class RC5_read:
    def __init__(self, pi, gpio):
        self.pi = pi
        self.gpio = gpio
        self.lt = 0
        self.code = 1
        self.bits = 1
        self.state = STATE_MID1
        self.last_fe = None
        self.last_re = None
        self.first_re = None
        self.last_call = None
        self.auto_modulated = True
        self.invert_logic = True

        pi.set_mode(gpio, pigpio.INPUT)
        if self.auto_modulated:
            self.cb = pi.callback(gpio, pigpio.EITHER_EDGE, self.cbf)
        else:
            self.cb = pi.callback(gpio, pigpio.RISING_EDGE, self.cbf)
        if self.invert_logic:
            self.wd = pi.set_watchdog(gpio, 0)
        else:
            self.wd = pi.set_watchdog(gpio, 10)

    def reset(self):
        self.code = 1
        self.bits = 1
        self.state = STATE_MID1
        self.last_re = None
        self.first_re = None

    def change_state(self, e):
        new_state = (TRANS_TABLE[self.state] >> e) & 0x3;
        if new_state == self.state:
            self.reset()
        else:
            self.state = new_state
            if self.state == STATE_MID0:
                self.code = (self.code << 1) + 0
                self.bits += 1
            elif self.state == STATE_MID1:
                self.code = (self.code << 1) + 1
                self.bits += 1

    def decode_segment(self, l, td):
        if td >= MIN_SHORT and td <= MAX_SHORT:
            if l:
                self.change_state(SHORT_PULSE)
            else:
                self.change_state(SHORT_SPACE)
        elif td >= MIN_LONG and td <= MAX_LONG:
            if l:
                self.change_state(LONG_PULSE)
            else:
                self.change_state(LONG_SPACE)
        elif td > MAX_LONG:
            self.reset()

    def cbf(self, g, l, t):
        if self.invert_logic:
            l = not l
        if self.auto_modulated:
            # save the first rising edge this segment
            if l == 1:
                if self.last_fe:
                    self.decode_segment(0, pigpio.tickDiff(self.last_fe, t))
                    self.last_fe = None
                self.last_re = t

            if l == 0 and self.last_re:
                self.decode_segment(1, pigpio.tickDiff(self.last_re, t))
                self.last_fe = t
                self.last_re = None
        else:
            # save the first rising edge this segment
            if l == 1 and not self.first_re:
                self.first_re = t
                self.last_re = t
                return

            # pulsing
            if l == 1 and pigpio.tickDiff(self.last_re, t) < 50:
                self.last_re = t
                return

            if self.first_re and self.last_re:
                self.decode_segment(1, pigpio.tickDiff(self.first_re, self.last_re))
                if not l == 2 and self.last_re:
                    self.decode_segment(0, pigpio.tickDiff(self.last_re, t))
                    self.first_re = t
                    self.last_re = t

        # got a full send
        if self.bits == 14:
            sys.stdout.write(chr(self.code & COMMAND_MASK))
            sys.stdout.flush()
            self.reset()

def main():
     pi = pigpio.pi('localhost', 9999)
     r = RC5_read(pi, 3)

     while True:
          time.sleep(1)

if __name__ == "__main__":
     main()
