#!/usr/bin/python3

#based off code from joan2937 @ https://github.com/joan2937/pigpio/issues/217

import pigpio, time

def carrier(gpio, frequency, micros, dutycycle=0.25):
    wf = []
    cycle = 1000000.0 / frequency
    cycles = int(round(micros/cycle))
    on = int(round(cycle * dutycycle))
    sofar = 0
    for c in range(cycles):
        target = int(round((c+1)*cycle))
        sofar += on
        off = target - sofar
        sofar += off
        wf.append(pigpio.pulse(1 << gpio, 0, on))
        wf.append(pigpio.pulse(0, 1 << gpio, off))
    return wf

class RC5:
    def __init__(self, pi, gpio, address=0):
        self.pi = pi
        self.gpio = gpio
        self.address = address & 15
        self.bip = bip(pi, gpio, 37900, 889, 889, True)

    def set_address(self, address):
        self.address = address & 15

    def send_raw(self, data, bits):
        chain = self.bip.format(data, bits)
        self.pi.wave_chain(chain)

    def send(self, command):
        command &= 255

        data = (3<<12) | (self.address << 8) | command
        self.send_raw(data, 14)

    def cancel(self):
        self.bip.cancel()

class bip:
    def __init__(self, pi, gpio, freq, mark, space, rising_1, duty=0.25):
        self.pi = pi
        self.gpio = gpio
        self.rising_1 = rising_1
        pi.set_mode(gpio, pigpio.OUTPUT)
        pi.wave_add_new()

        # mark
        pi.wave_add_generic(carrier(gpio, freq, mark, duty))
        #pi.wave_add_generic([pigpio.pulse(1 << gpio, 0, space)])
        self.w_mark = pi.wave_create()

        # space
        pi.wave_add_generic([pigpio.pulse(0, 1 << gpio, space)])
        self.w_space = pi.wave_create()

        if rising_1:
            self.bit = [[self.w_mark, self.w_space], [self.w_space, self.w_mark]]
        else:
            self.bit = [[self.w_space, self.w_mark], [self.w_mark, self.w_space]]

    def format(self, data, bits):
        chain = []
        for i in range(bits-1, -1, -1):
            chain += self.bit[(data>>i) & 1]
        chain += [ self.w_space ]
        return chain

    def cancel(self):
        if self.w_mark is not None:
            self.pi.wave_delete(self.w_mark)
            self.w_mark = None

        if self.w_space is not None:
            self.pi.wave_delete(self.w_space)
            self.w_space = None

def main():
    pi = pigpio.pi()
    pi.wave_clear()
    sender = RC5(pi, 2)
    sender.set_address(5)

    msg = "Never gonna give you up, never gonna let you down!\n"
    while 1:
        for i in msg:
            sender.send(ord(i))
            time.sleep(0.04)
    return 1

if __name__ == "__main__":
     main()
