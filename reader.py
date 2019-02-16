#!/usr/bin/python3

import pigpio, time

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

S2_MASK = 0x1000
S2_SHIFT = 12
TOGGLE_MASK = 0x0800
TOGGLE_SHIFT = 11
ADDRESS_MASK = 0x7c0
ADDRESS_SHIFT = 6
COMMAND_MASK = 0x003f
COMMAND_SHIFT = 0

class RC5_read:
    def __init__(self, pi, gpio):
        self.pi = pi
        self.gpio = gpio
        self.lt = 0
        self.code = 1
        self.bits = 1
        self.state = STATE_MID1
        self.last_fe = 0
        self.first_re = None

        pi.set_mode(gpio, pigpio.INPUT)
        self.cb = pi.callback(gpio, pigpio.EITHER_EDGE, self.cbf)
        self.wd = pi.set_watchdog(gpio, 10)

    def reset(self):
        self.code = 1
        self.bits = 1
        self.state = STATE_MID1
        self.last_fe = 0
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

    def decodeP(self, l, td):
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
            print('reset')
            self.reset()

    def cbf(self, g, l, t):
        # save last falling edge
        if not l:
            self.last_fe = t
            return

        # save the first rising edge this segment
        if l == 1 and not self.first_re:
            self.first_re = t
            return

        # check if we're pulsing
        if pigpio.tickDiff(self.last_fe, t) < 50:
            return

        # decode the last 2 segments
        if self.first_re and self.last_fe:
            self.decodeP(1, pigpio.tickDiff(self.first_re, self.last_fe))
        if not l == 2:
            self.decodeP(0, pigpio.tickDiff(self.last_fe, t))
            self.first_re = t

        # got a full send
        if self.bits >= 14:
            code = (self.code >> (self.bits - 14))
            print(hex(code & COMMAND_MASK))
            self.reset()

def main():
     pi = pigpio.pi()
     r = RC5_read(pi, 3)

     while True:
          time.sleep(1)

if __name__ == "__main__":
     main()
