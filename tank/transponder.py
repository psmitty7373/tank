#!/usr/bin/python3

import json, pigpio, sys, time, queue
from uuid import getnode as get_mac
from hashlib import md5
import websocket
from threading import Thread

ID = int(md5(str(hex(get_mac())).encode('ascii')).hexdigest()[0:6], 16)

class Transponder:
    def __init__(self, pi, gpio, fps=40):
        self.pi = pi
        self.pi.wave_clear()
        pi.wave_add_new()
        pi.wave_add_generic([pigpio.pulse(1 << gpio, 0, 25000)])
        self.w_mark = pi.wave_create()
        pi.wave_add_generic([pigpio.pulse(0, 1 << gpio, 25000)])
        self.w_space = pi.wave_create()

    def beacon(self):
        chain = [255, 0] + [self.w_mark,  self.w_space] + [255, 3]
        self.pi.wave_chain(chain)

    def ident(self):
        chain = [255, 0] + [self.w_mark] * 2 + [self.w_space] * 8 + [255, 1, 8, 0] + [255, 0] + [self.w_mark] * 5 + [self.w_space] * 5 + [255, 3]
        self.pi.wave_chain(chain)

class Transponder_Socket(Thread):
    def __init__(self, q):
        Thread.__init__(self)
        self.q = q
        self.running = True
        #websocket.enableTrace(True)

    def initiate(self):
        self.ws = websocket.WebSocketApp("ws://192.168.97.130:8000/ws",
                on_message = lambda ws, msg: self.on_message(ws, msg),
                on_error = lambda ws, error: self.on_error(ws, error),
                on_close = lambda ws: self.on_close(ws),
                on_open = lambda ws: self.on_open(ws))
        self.ws.run_forever()

    def on_message(self, ws, msg):
        self.q.put(json.loads(msg))

    def on_error(self, ws, error):
        print(error)

    def on_close(self, ws):
        print("### closed ###")
        # Attemp to reconnect with 2 seconds interval
        time.sleep(2)
        self.initiate()

    def ws_loop(self, *args):
        while self.running:
            # Sending message with 1 second intervall
            time.sleep(1)
            msg = {'t': 'hb', 'tid': ID }
            self.ws.send(json.dumps(msg))
        time.sleep(1)
        self.ws.close()
        print("thread terminating...")

    def on_open(self, ws):
        print("### Initiating new websocket connection ###")
        self.ws.send(json.dumps({'t':'join', 'tid': ID}))
        self.thread = Thread(target=self.ws_loop)
        self.thread.start()

    def run(self):
        self.initiate()

def main():
    print("Started.")
    q = queue.Queue()
    ws = Transponder_Socket(q)
    ws.start()
    hit = 0
    pi = pigpio.pi("localhost", 9999)
    t_led = Transponder(pi, 22)
    t_led.ident()

    while True:
        while not q.empty():
            msg = q.get()
            print(msg)
            if 't' in msg.keys():
                if msg['t'] == 'poll':
                    print('IDENT')
                    t_led.ident()
        time.sleep(0.01)

    t_led.running = False
    t_led.join()

if __name__ == "__main__":
     main()
