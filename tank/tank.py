#!/usr/bin/python3

import json, math, pickle, pigpio, os, signal, struct, time
from hashlib import md5
from multiprocessing import Process
from multiprocessing import Queue
from multiprocessing import set_start_method
import tornado.ioloop
import tornado.web as web
import tornado.websocket
from tornado import gen
from threading import Thread
from uuid import getnode as get_mac

running = True

# settings
right_pwm_f = 19
right_pwm_r = 26

left_pwm_f = 27
left_pwm_r = 17

min_throttle = 50.0

imu_enabled = False
current_enabled = False
server_url = "ws://192.168.97.130:8000/ws"

ID = int(md5(str(hex(get_mac())).encode('ascii')).hexdigest()[0:6], 16)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def steering(x, y):
    if y >= 0: # forward
        if x >= 0: # right
            premix_l = 1.0
            premix_r = 1.0 - 1.5 * x
        else: # left
            premix_l = 1.0 + 1.5 * x
            premix_r = 1.0
    else: # backward
        if x >= 0: # right
            premix_l = 1.0 - x
            premix_r = 1.0
        else: # left
            premix_l = 1.0
            premix_r = 1.0 + x

    premix_l = premix_l * y
    premix_r = premix_r * y

    piv_speed = 0.15 * x
    if abs(y) > 0.3:
        piv_scale = 0.0
    else:
        piv_scale = 1.0 - abs(y) / 0.3

    l = (1.0 - piv_scale) * premix_l + piv_scale * piv_speed
    r = (1.0 - piv_scale) * premix_r + piv_scale * -piv_speed


    # translate to pwm
    r = translate(r, -1, 1, -255, 255)
    l = translate(l, -1, 1, -255, 255)
    if abs(r) > 0:
        r = math.copysign(translate(abs(r), 0, 255, min_throttle, 255), r)
    if abs(l) > 0:
        l = math.copysign(translate(abs(l), 0, 255, min_throttle, 255), l)
    return (round(r), round(l))

class Tank(Process):
    import laser
    import BNO055
    import INA219
    def __init__(self, to_tank, to_tornado, to_main):
        super(Tank, self).__init__()
        self.game_config = {'corners': {'tl': [0,0], 'tr': [640,0], 'bl': [0, 480], 'br': [640, 480]}, 'map_features': []}
        self.objects = {}
        self.to_tank = to_tank
        self.to_main = to_main
        self.to_tornado = to_tornado
       
    def shutdown(self, signal, frame):
        print('Stopping tank.')
        self.running = False

    def run(self):
        print('Starting tank.')
        self.running = True

        # init signal handler
        signal.signal(signal.SIGINT, self.shutdown)

        #init pigpio
        self.pi = pigpio.pi('localhost', 9999)

        #init weapon
        self.weapon = self.laser.laser(self.pi, 25)
        self.weapon_ready = True

        #init imu
        if imu_enabled:
            self.bno = self.BNO055.BNO055()
            if self.bno.begin() is not True:
                print("Error initializing device")
                return False
            self.bno.setExternalCrystalUse(True)
            if os.path.isfile('imu.conf'):
                print("Loaded imu.conf")
                with open('imu.conf','rb') as f:
                    cd = pickle.load(f)
                    self.bno.setCalibrationData(cd)
        else:
            self.bno = None

        #init ina219
        if current_enabled:
            self.ina = self.INA219.INA219(bus=3, address=0x44)

        #init transponder
        self.trans = Transponder(self.pi, 22)
        self.trans.beacon()

        #right
        self.pi.set_mode(right_pwm_f, pigpio.OUTPUT)
        self.pi.set_mode(right_pwm_r, pigpio.OUTPUT)
        self.right_dir = 'f'
        self.right_braking = False
        self.right_brake_time = time.time() * 1000
        self.r_t = 0

        #left
        self.pi.set_mode(left_pwm_f, pigpio.OUTPUT)
        self.pi.set_mode(left_pwm_r, pigpio.OUTPUT)
        self.left_dir = 'f'
        self.left_braking = False
        self.left_brake_time = time.time() * 1000
        self.l_t = 0

        # ina219
        self.current = 0
        self.volts = 0

        # timers
        self.brake_time = 100
        self.temp_time = time.time() * 1000
        self.failsafe_time = time.time() * 1000
        self.heartbeat_time = time.time() * 1000
        self.weapon_reload_time = 0

        # position
        self.pos_ready = False
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.ppos_x = 0
        self.ppos_y = 0
        self.ppos_z = 0
        self.posm_x = []
        self.posm_y = []
        self.posm_z = []
        self.pos_quality = 0
        self.azim_x = 0
        self.azim_y = 0
        self.azim_z = 0

        # messaging

        print("Running!")
        while self.running:
            curr_time = time.time() * 1000

            # if no update in a second, stop motors
            if curr_time - self.failsafe_time > 1000:
                self.pi.set_PWM_dutycycle(left_pwm_f, 0)
                self.pi.set_PWM_dutycycle(left_pwm_r, 0)
                self.pi.set_PWM_dutycycle(right_pwm_f, 0)
                self.pi.set_PWM_dutycycle(right_pwm_r, 0)

            # update reload timer
            if not self.weapon_ready:
                if curr_time - self.weapon_reload_time > 5000:
                    self.weapon_ready = True

            # IMU --------------------------------------------------------------------
            # poll imu
            if imu_enabled and self.bno:
                self.azim_x, self.azim_y, self.azim_z = self.bno.getVector(self.BNO055.BNO055.VECTOR_EULER)
                calib = self.bno.getCalibrationStatus()
                if curr_time - self.temp_time > 30000:
                    if calib[0] == 3 and calib[1] == 3 and calib[3] == 3:
                        cd = self.bno.getCalibrationData()
                        with open('imu.conf', 'wb') as f:
                            pickle.dump(cd, f)
                        self.temp_time = curr_time

            # INA219 ----------------------------------------------------------------
            if current_enabled and self.ina:
                self.volts = self.ina.get_bus_voltage()
                self.current = self.ina.get_current()

            # handle all messages in the queue
            while not self.to_tank.empty():
                self.failsafe_time = curr_time
                msg = self.to_tank.get()

                if 't' in msg.keys():
                    # throttle
                    if msg['t'] == "throttle" and 'throttle' in msg.keys():
                        self.r_t, self.l_t = msg['throttle']

                    # fire
                    elif msg['t'] == "fire_weapon":
                        self.weapon_reload_time = curr_time
                        self.weapon_ready = False
                        self.weapon.send(ord('A'))

                    # shutdown
                    elif msg['t'] == "shutdown":
                        self.running = False

                    # game tick
                    elif msg['t'] == 'tick':
                        if 'tanks' in msg.keys():
                           tanks = json.loads(msg['tanks'])
                           for t in tanks.keys():
                               if int(t) == ID:
                                   self.pos_x = tanks[t]['pos'][0]
                                   self.pos_y = tanks[t]['pos'][1]
                                   self.pos_z = 0
                        
                        if 'objects' in msg.keys():
                            self.objects = json.loads(msg['objects'])

                    # handle a poll request
                    elif msg['t'] == 'poll':
                        self.trans.poll()

                    # if we get an game_config, blast it out
                    elif msg['t'] == 'game_config' and 'config' in msg.keys():
                        self.game_config = msg['config']
                        self.to_tornado.put(msg)

                    # when a client connects send the current game_config
                    elif msg['t'] == 'connect':
                        self.to_tornado.put({ 't': 'game_config', 'config': self.game_config })

                # right motor
                if self.r_t > 0: # forward
                    # changing directions
                    if self.right_dir == 'b':
                        # start braking
                        if not self.right_braking:
                            self.right_braking = True
                            self.right_brake_time = time.time() * 1000

                        # done braking
                        if time.time() * 1000 - self.right_brake_time > self.brake_time:
                            self.right_braking = False
                            self.right_dir = 'f'

                    # same dir, cancel braking
                    elif self.right_braking:
                        self.right_braking = False

                elif self.r_t < 0:
                    if self.right_dir == 'f':
                        # start braking
                        if not self.right_braking:
                            self.right_braking = True
                            self.right_brake_time = time.time() * 1000

                        # done braking
                        if time.time() * 1000 - self.right_brake_time > self.brake_time:
                            self.right_braking = False
                            self.right_dir = 'b'

                    # same dir, cancel braking
                    elif self.right_braking:
                        self.right_braking = False

                # left motor
                if self.l_t > 0: # forward
                    # changing directions
                    if self.left_dir == 'b':
                        # start braking
                        if not self.left_braking:
                            self.left_braking = True
                            self.left_brake_time = time.time() * 1000

                        # done braking
                        if time.time() * 1000 - self.left_brake_time > self.brake_time:
                            self.left_braking = False
                            self.left_dir = 'f'

                    # same dir, cancel braking
                    elif self.left_braking:
                        self.left_braking = False

                elif self.l_t < 0:
                    if self.left_dir == 'f':
                        # start braking
                        if not self.left_braking:
                            self.left_braking = True
                            self.left_brake_time = time.time() * 1000

                        # done braking
                        if time.time() * 1000 - self.left_brake_time > self.brake_time:
                            self.left_braking = False
                            self.left_dir = 'b'

                    # same dir, cancel braking
                    elif self.left_braking:
                        self.left_braking = False

                #left motor speed
                if self.l_t == 0 or self.left_braking:
                    self.pi.set_PWM_dutycycle(left_pwm_f, 0)
                    self.pi.set_PWM_dutycycle(left_pwm_r, 0)
                else:
                    if self.left_dir == 'f':
                        self.pi.set_PWM_dutycycle(left_pwm_r, 0)
                        self.pi.set_PWM_dutycycle(left_pwm_f, abs(self.l_t))
                    else:
                        self.pi.set_PWM_dutycycle(left_pwm_f, 0)
                        self.pi.set_PWM_dutycycle(left_pwm_r, abs(self.l_t))

                # right motor speed
                if self.r_t == 0 or self.right_braking:
                    self.pi.set_PWM_dutycycle(right_pwm_f, 0)
                    self.pi.set_PWM_dutycycle(right_pwm_r, 0)
                else:
                    if self.right_dir == 'f':
                        self.pi.set_PWM_dutycycle(right_pwm_r, 0)
                        self.pi.set_PWM_dutycycle(right_pwm_f, abs(self.r_t))
                    else:
                        self.pi.set_PWM_dutycycle(right_pwm_f, 0)
                        self.pi.set_PWM_dutycycle(right_pwm_r, abs(self.r_t))

            # send current status
            if not self.to_tornado.full() and time.time() * 1000 - self.heartbeat_time > 100:
                self.heartbeat_time = time.time() * 1000
                self.to_tornado.put({ 't': 'status', 'current': self.current, 'volts': self.volts, 'l_t': self.l_t, 'r_t': self.r_t, 'x': self.pos_x, 'y': self.pos_y, 'z': self.pos_z, 'a_x': self.azim_x, 'a_y': self.azim_y, 'a_z': self.azim_z, 'quality': self.pos_quality, 'objects': self.objects })

            # prevent busy waiting
            time.sleep(0.01)

        print('Tank done.')
        self.pi.stop()


class Transponder:
    def __init__(self, pi, gpio, fps=40, clear=False):
        self.pi = pi
        self.pi.write(gpio, 1)
        if clear:
            self.pi.wave_clear()
        pi.wave_add_new()
        pi.wave_add_generic([pigpio.pulse(1 << gpio, 0, 25000)])
        self.w_mark = pi.wave_create()
        pi.wave_add_generic([pigpio.pulse(0, 1 << gpio, 25000)])
        self.w_space = pi.wave_create()

    def beacon(self):
        chain = [255, 0] + [self.w_mark] * 8 + [self.w_space] * 2 + [255, 3]
        self.pi.wave_chain(chain)

    def poll(self):
        chain = [255, 0] + [self.w_mark] * 4 + [self.w_space] * 6 + [255, 1, 4, 0] + [255, 0] + [self.w_mark] * 8 + [self.w_space] * 2 + [255, 3]
        self.pi.wave_chain(chain)


class WebsocketClient(Thread):
    def __init__(self, to_client, to_tank, to_main):
        super(WebsocketClient, self).__init__()
        self.ioloop = tornado.ioloop.IOLoop.instance()
        self.ws = None
        self.to_client = to_client
        self.to_main = to_main
        self.to_tank = to_tank
        self.connect()
        tornado.ioloop.PeriodicCallback(self.update_socket, 1000).start()
        self.ioloop.start()

    @gen.coroutine
    def connect(self):
        global running
        try:
            self.ws = yield tornado.websocket.websocket_connect(server_url, on_message_callback=self.on_message)
        except:
            if running:
                time.sleep(2)
                self.connect()
            else:
                self.ioloop.stop()
        else:
            self.send(json.dumps({ 't':'join', 'tid': ID }))

    def on_message(self, msg):
        if msg == None:
            if self.ws:
                self.ws.close()
            self.ws = None
            time.sleep(2)
            self.connect()
        else:
            self.to_tank.put(json.loads(msg))

    def send(self, msg):
        if not self.ws == None:
            self.ws.write_message(msg)
            return True
        return False

    def stop(self):
        self.ioloop.stop()

    def update_socket(self):
        global running
        if running:
            self.send(json.dumps({ 't': 'hb', 'tid': ID }))
        else:
            print('Stopping websocket client.')
            self.ioloop.stop()


class Webserver(Process):
    def __init__(self, to_tornado, to_tank, to_main):
        super(Webserver, self).__init__()
        self.to_tornado = to_tornado
        self.to_main = to_main
        self.to_tank = to_tank

    class MainHandler(tornado.web.RequestHandler):
        def get(self):
            self.render("index.html")

    class MyStaticFileHandler(tornado.web.StaticFileHandler):
        def set_extra_headers(self, path):
            self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

    class WebSocket(tornado.websocket.WebSocketHandler):

        def initialize(self, to_tornado, to_tank, to_main, connections):
            self.to_tornado = to_tornado
            self.to_main = to_main
            self.to_tank = to_tank
            self.connections = connections

        def open(self):
            self.connections.add(self)
            self.to_tank.put({ 't': 'connect' })
        
        def on_message(self, msg):
            msg = json.loads(msg)
            if msg['t'] == 't':
                r, l = steering(msg['x'], msg['y'])
                self.to_tank.put({ 't': 'throttle', 'throttle': (r,l) })
            elif msg['t'] == 'f':
                self.to_tank.put({ 't': 'fire_weapon' })
            while not self.to_tornado.empty():
                msg = self.to_tornado.get()
                [client.write_message(json.dumps(msg)) for client in self.connections]

        def on_close(self):
            self.connections.remove(self)

    def update_socket(self):
        if not self.running:
            [client.close() for client in self.connections]

    def signal_handler(self, signal, frame):
        print("Stopping Tornado.")
        self.running = False
        tornado.ioloop.IOLoop.current().stop()

    def run(self):
        self.running = True
        print('Starting webserver.')
        signal.signal(signal.SIGINT, self.signal_handler)

        self.connections = set()

        public = os.path.join(os.path.dirname(__file__), 'tank_public')

        self.webapp = tornado.web.Application([
            (r"/tank_ws", self.WebSocket, {'to_tornado': self.to_tornado, 'to_main': self.to_main, 'to_tank': self.to_tank, 'connections': self.connections }),
            (r'/(.*)', self.MyStaticFileHandler, {'path': public, "default_filename": "index.html" }),
        ])

        self.webapp.listen(8000)
        tornado.ioloop.PeriodicCallback(self.update_socket, 1000).start()
        tornado.ioloop.IOLoop.current().start()

def signal_handler(signal, frame):
    global running
    print("Stopping.")
    running = False
    return

def main():
    print('Starting Tank 1.0.')
    signal.signal(signal.SIGINT, signal_handler)

    set_start_method('spawn')

    to_tornado = Queue()
    to_tank = Queue()
    to_client = Queue()
    to_main =Queue()

    # multiprocs
    tank = Tank(to_tank, to_tornado, to_main)
    tank.start()

    web = Webserver(to_tornado, to_tank, to_main)
    web.start()

    # threads
    cs = WebsocketClient(to_client, to_tank, to_main)
    cs.start()
    cs.join()

    print('Shut down.')

if __name__ == "__main__":
    main()
