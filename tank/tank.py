#!/usr/bin/python3

import configparser, json, math, pickle, pigpio, os, signal, struct, time
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

import cProfile as profile

running = True


# load settings
ID = int(md5(str(hex(get_mac())).encode('ascii')).hexdigest()[0:6], 16)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def steering(x, y, min_throttle=50.0):
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
        self.tank_config = configparser.ConfigParser()
        self.init_config()
       
    def shutdown(self, signal, frame):
        print('Stopping tank.')
        self.running = False

    def init_config(self):
        default_config = {
            'imu_enabled': { 'default': True, 'type': bool, 'check': lambda x: x == 'True' or x == 'False' },
            'current_enabled': { 'default': True, 'type': bool, 'check': lambda x: x == 'True' or x == 'False' },
            'min_throttle': { 'default': 50.0, 'type': float, 'check': lambda x: float(x) >= 0 and float(x) <= 75 },
            'right_pwm_f': { 'default': 27, 'type': int, 'check': lambda x: int(x) >= 1 and float(x) <= 29 },
            'right_pwm_r': { 'default': 22, 'type': int, 'check': lambda x: int(x) >= 1 and float(x) <= 29 },
            'left_pwm_f': { 'default': 18, 'type': int, 'check': lambda x: int(x) >= 1 and float(x) <= 29 },
            'left_pwm_r': { 'default': 17, 'type': int, 'check': lambda x: int(x) >= 1 and float(x) <= 29 }
        }

        if not os.path.exists('tank.conf'):
            for k in default_config.keys():
                self.tank_config['tank'] = default_config[k]['default']
            self.tank_config.write(open('tank.conf', 'w'))

        else:
            self.tank_config.read('tank.conf')
            # verify existing config options
            for k in self.tank_config['tank'].keys():
                print(k, self.tank_config['tank'][k])
                if k in default_config.keys():
                    try:
                        if default_config[k]['check'](self.tank_config['tank'][k]):
                            continue
                        else:
                            raise
                    except:
                        print('Invalid config option:', k, ' Using default value.')
                        self.tank_config.remove_option('tank', k)

            # add missing defaults
            for k in default_config.keys():
                if k not in self.tank_config['tank'].keys():
                    self.tank_config['tank'][k] = str(default_config[k]['default'])

        self.right_pwm_f = int(self.tank_config['tank']['right_pwm_f'])
        self.right_pwm_r = int(self.tank_config['tank']['right_pwm_r'])

        self.left_pwm_f = int(self.tank_config['tank']['left_pwm_f'])
        self.left_pwm_r = int(self.tank_config['tank']['left_pwm_r'])

        self.min_throttle = float(self.tank_config['tank']['min_throttle'])

        self.imu_enabled = self.tank_config['tank']['imu_enabled'] == 'True'
        self.current_enabled = self.tank_config['tank']['current_enabled'] == 'True'


    #def run(self):
    #    profile.runctx('self.run2()', globals(), locals())

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
        if self.imu_enabled:
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
        if self.current_enabled:
            self.ina = self.INA219.INA219(bus=3, address=0x44)

        #init transponder
        self.trans = Transponder(self.pi, 12)
        self.trans.beacon()

        curr_time = time.time() * 1000

        #right
        self.pi.set_mode(self.right_pwm_f, pigpio.OUTPUT)
        self.pi.set_mode(self.right_pwm_r, pigpio.OUTPUT)
        right_dir = 'f'
        right_braking = False
        right_brake_time = curr_time
        r_t = 0
        last_r_t_f = 0
        last_r_t_r = 0

        #left
        self.pi.set_mode(self.left_pwm_f, pigpio.OUTPUT)
        self.pi.set_mode(self.left_pwm_r, pigpio.OUTPUT)
        left_dir = 'f'
        left_braking = False
        left_brake_time = curr_time
        l_t = 0
        last_l_t_f = 0
        last_l_t_r = 0

        # ina219
        self.current = 0
        self.volts = 0

        # timers
        brake_time = 100
        t30s_time = curr_time
        failsafe_time = curr_time
        t100ms_time = curr_time
        weapon_reload_time = 0

        # position
        self.pos_ready = False
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.pos_quality = 0
        self.azim_x = 0
        self.azim_y = 0
        self.azim_z = 0

        # messaging

        print("Running!")
        while self.running:
            curr_time = time.time() * 1000

            # if no update in a second, stop motors
            if curr_time - failsafe_time > 1000:
                last_l_t_f = last_l_t_r = 0
                last_r_t_f = last_r_t_r = 0
                self.pi.set_PWM_dutycycle(self.left_pwm_f, 0)
                self.pi.set_PWM_dutycycle(self.left_pwm_r, 0)
                self.pi.set_PWM_dutycycle(self.right_pwm_f, 0)
                self.pi.set_PWM_dutycycle(self.right_pwm_r, 0)

            # update reload timer
            if not self.weapon_ready:
                if curr_time - self.weapon_reload_time > 5000:
                    self.weapon_ready = True

            # poll sensors
            if curr_time - t100ms_time > 100:
                # IMU --------------------------------------------------------------------
                # poll imu
                if self.imu_enabled and self.bno:
                    self.azim_x, self.azim_y, self.azim_z = self.bno.getVector(self.BNO055.BNO055.VECTOR_EULER)
                    calib = self.bno.getCalibrationStatus()
                    if curr_time - t30s_time > 30000:
                        if calib[0] == 3 and calib[1] == 3 and calib[3] == 3:
                            cd = self.bno.getCalibrationData()
                            with open('imu.conf', 'wb') as f:
                                pickle.dump(cd, f)
                            t30s_time = curr_time

                # INA219 ----------------------------------------------------------------
                if self.current_enabled and self.ina:
                    self.volts = self.ina.get_bus_voltage()
                    self.current = self.ina.get_current()

            # handle all messages in the queue
            while not self.to_tank.empty():
                failsafe_time = curr_time
                msg = self.to_tank.get()


                if 't' in msg.keys():
                    # fire
                    if msg['t'] == "fire_weapon":
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

                    # update tank config
                    elif msg['t'] == 'update_tank_config' and 'config' in msg.keys():
                        for k in msg['config'].keys():
                            self.tank_config['tank'][k] = msg['config'][k]

                        # save the config
                        self.tank_config.write(open('tank.conf', 'w'))

                        # activate the config
                        self.init_config()

                        # send the new config
                        self.to_tornado.put({ 't': 'tank_config', 'config': dict(self.tank_config['tank']) })

                    # if we get an game_config, blast it out
                    elif msg['t'] == 'game_config' and 'config' in msg.keys():
                        self.game_config = msg['config']
                        self.to_tornado.put(msg)

                    # when a client connects send the current game_config
                    elif msg['t'] == 'connect':
                        self.to_tornado.put({ 't': 'game_config', 'config': self.game_config })
                        self.to_tornado.put({ 't': 'tank_config', 'config': dict(self.tank_config['tank']) })

                    # throttle
                    elif msg['t'] == "throttle" and 'throttle' in msg.keys():
                        r_t, l_t = msg['throttle']

                        # right motor
                        if r_t > 0: # forward
                            # changing directions
                            if right_dir == 'b':
                                # start braking
                                if not right_braking:
                                    right_braking = True
                                    right_brake_time = curr_time

                                # done braking
                                if curr_time - right_brake_time > brake_time:
                                    right_braking = False
                                    right_dir = 'f'

                            # same dir, cancel braking
                            elif right_braking:
                                right_braking = False

                        elif r_t < 0:
                            if right_dir == 'f':
                                # start braking
                                if not right_braking:
                                    right_braking = True
                                    right_brake_time = curr_time

                                # done braking
                                if curr_time - right_brake_time > brake_time:
                                    right_braking = False
                                    right_dir = 'b'

                            # same dir, cancel braking
                            elif right_braking:
                                right_braking = False

                        # left motor
                        if l_t > 0: # forward
                            # changing directions
                            if left_dir == 'b':
                                # start braking
                                if not left_braking:
                                    left_braking = True
                                    left_brake_time = curr_time

                                # done braking
                                if curr_time - left_brake_time > brake_time:
                                    left_braking = False
                                    left_dir = 'f'

                            # same dir, cancel braking
                            elif left_braking:
                                left_braking = False

                        elif l_t < 0:
                            if left_dir == 'f':
                                # start braking
                                if not left_braking:
                                    left_braking = True
                                    left_brake_time = curr_time

                                # done braking
                                if curr_time - left_brake_time > brake_time:
                                    left_braking = False
                                    left_dir = 'b'

                            # same dir, cancel braking
                            elif left_braking:
                                left_braking = False

                        #left motor speed
                        if l_t == 0 or left_braking:
                            if last_l_t_f != 0:
                                self.pi.set_PWM_dutycycle(self.left_pwm_f, 0)
                                last_l_t_f = 0

                            if last_l_t_r != 0:
                                self.pi.set_PWM_dutycycle(self.left_pwm_r, 0)
                                last_l_t_r = 0

                        else:
                            if left_dir == 'f':
                                if last_l_t_f != abs(l_t):
                                    self.pi.set_PWM_dutycycle(self.left_pwm_f, abs(l_t))
                                    last_l_t_f = abs(l_t)

                                if last_l_t_r != 0:
                                    self.pi.set_PWM_dutycycle(self.left_pwm_r, 0)
                                    last_l_t_r = 0

                            else:
                                if last_l_t_f != 0:
                                    self.pi.set_PWM_dutycycle(self.left_pwm_f, 0)
                                    last_l_t_f = 0

                                if last_l_t_r != abs(l_t):
                                    self.pi.set_PWM_dutycycle(self.left_pwm_r, abs(l_t))
                                    last_l_t_r = abs(l_t)

                        # right motor speed
                        if r_t == 0 or right_braking:
                            self.pi.set_PWM_dutycycle(self.right_pwm_f, 0)
                            self.pi.set_PWM_dutycycle(self.right_pwm_r, 0)
                        else:
                            if right_dir == 'f':
                                self.pi.set_PWM_dutycycle(self.right_pwm_r, 0)
                                self.pi.set_PWM_dutycycle(self.right_pwm_f, abs(r_t))
                            else:
                                self.pi.set_PWM_dutycycle(self.right_pwm_f, 0)
                                self.pi.set_PWM_dutycycle(self.right_pwm_r, abs(r_t))

            # send current status
            if not self.to_tornado.full() and curr_time - t100ms_time > 100:
                t100ms_time = curr_time
                self.to_tornado.put({ 't': 'status', 'current': self.current, 'volts': self.volts, 'l_t': l_t, 'r_t': r_t, 'x': self.pos_x, 'y': self.pos_y, 'z': self.pos_z, 'a_x': self.azim_x, 'a_y': self.azim_y, 'a_z': self.azim_z, 'quality': self.pos_quality, 'objects': self.objects })

            # prevent busy waiting
            time.sleep(0.05)

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
        self.server_config = configparser.ConfigParser()
        self.init_config()
        self.ioloop = tornado.ioloop.IOLoop.instance()
        self.ws = None
        self.to_client = to_client
        self.to_main = to_main
        self.to_tank = to_tank
        self.connect()
        tornado.ioloop.PeriodicCallback(self.update_socket, 1000).start()
        self.ioloop.start()

    def init_config(self):
        default_config = {
            'server_url': 'ws://10.0.0.2:8000/ws'
        }

        if not os.path.exists('server.conf'):
            self.server_config['server'] = default_config
            self.server_config.write(open('server.conf', 'w'))
        else:
            self.server_config.read('server.conf')
            for k in default_config.keys():
                if k not in self.server_config['server'].keys():
                    self.server_config['server'][k] = default_config[k]

    @gen.coroutine
    def connect(self):
        global running
        try:
            self.ws = yield tornado.websocket.websocket_connect(self.server_config['server']['server_url'], on_message_callback=self.on_message)
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
            # process messages from the websocket
            msg = json.loads(msg)
            # throttle update
            if msg['t'] == 't':
                r, l = steering(msg['x'], msg['y'])
                self.to_tank.put({ 't': 'throttle', 'throttle': (r,l) })

            # tank config update
            elif msg['t'] == 'update_tank_config':
                self.to_tank.put(msg)

            # fire weapon
            elif msg['t'] == 'f':
                self.to_tank.put({ 't': 'fire_weapon' })
            
            # process messages from other subprocesses to the websocket
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
