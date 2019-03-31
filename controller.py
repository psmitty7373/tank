#!/usr/bin/python3

import json, math, pickle, pigpio, pfilter, os, signal, struct, time
from hashlib import md5
from multiprocessing import Process
from multiprocessing import Queue
from multiprocessing import set_start_method
from functools import partial
from statistics import median
import tornado.ioloop
import tornado.web as web
import tornado.websocket
from tornado import gen
from threading import Thread
from uuid import getnode as get_mac

right_pwm_f = 17
right_pwm_r = 27

left_pwm_f = 23
left_pwm_r = 24

min_throttle = 50.0

serial_port_enabled = False
serial_port_mode = "UWB"
serial_port_baud = 115200
serial_port = "/dev/serial0"
imu_enabled = True

ID = int(md5(str(hex(get_mac())).encode('ascii')).hexdigest()[0:6], 16)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def steering2(x, y):
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
    def __init__(self, to_tank, to_tornado, to_main):
        super(Tank, self).__init__()
        self.to_tank = to_tank
        self.to_main = to_main
        self.to_tornado = to_tornado
       
    def shutdown(self, signal, frame):
        print('Stopping tank.')
        self.running = False

    def update_uwb(self):
        if not serial_port_enabled or not serial_port_mode == "UWB":
            return
        rlv_type = 0
        rlv_payload = b""
        num_bytes, rx_bytes = self.pi.serial_read(self.s1, 2)
        if num_bytes == 2:
            rlv_type, rlv_payload_len = struct.unpack('<bb', rx_bytes)
            if rlv_payload_len > 0:
                num_bytes, rlv_payload = self.pi.serial_read(self.s1, rlv_payload_len)
        elif num_bytes == 0:
            self.pi.serial_write(self.s1, '\x32\x00')
        if not rlv_type == 0:
            self.process_rlv_pkt({ 'rlv_type': rlv_type, 'rlv_payload': rlv_payload })

    def process_rlv_pkt(self, pkt):
        print(pkt)
        if pkt['rlv_type'] == 0:
            return
        elif pkt['rlv_type'] == 0x5a:
            status = struct.unpack('<b', pkt['rlv_payload'])[0]
            if status & 1:
                self.pos_ready = True
        elif pkt['rlv_type'] == 65 and len(pkt['rlv_payload']) == 13:
            self.pos_x, self.pos_y, self.pos_z, self.pos_quality = struct.unpack('<lllb', pkt['rlv_payload'])
            self.posm_x.append(self.pos_x)
            self.posm_y.append(self.pos_y)
            self.posm_z.append(self.pos_z)
            # median position filter
            if len(self.posm_x) > 5:
                self.posm_x = self.posm_x[-5:]
                self.posm_y = self.posm_y[-5:]
                self.posm_z = self.posm_z[-5:]
            self.pos_x = self.pos_x / 1000.0
            self.pos_y = self.pos_y / 1000.0
            self.pos_z = self.pos_z / 1000.0
        elif pkt['rlv_type'] == 73:
            num_distances = struct.unpack('<b', pkt['rlv_payload'][:1])[0]
            for i in range(0, num_distances):
                uwb_addr, distance, quality, pos_x, pos_y, pos_z, pos_quality = struct.unpack('<Hlblllb', pkt['rlv_payload'][1 + i * 20: 1 + (i+1) * 20])
                self.pf.addRangeMeasurement(uwb_addr, [pos_x/1000.0, pos_y/1000.0, pos_z/1000.0], distance/1000.0, 0.3)

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

        if serial_port_enabled:
            print("Opening serial port.")
            self.s1 = self.pi.serial_open(serial_port, serial_port_baud)
        else:
            self.s1 = None

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

        #particle filter
        self.pf = pfilter.ParticleFilter(200, 0.1, (10, 10, 10))

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

            # UWB --------------------------------------------------------------------
            # poll uwb serial port
            self.update_uwb()

            # position info ready
            if self.pos_ready:
                self.pos_ready = False
                self.pi.serial_write(self.s1, '\x02\x00') #get curr position
#                self.pi.serial_write(self.s1, '\x0c\x00') #get measurements
                #self.pf.update()
                self.ppos_x, self.ppos_y, self.ppos_z = self.pf.getEstimate()

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

            while not self.to_tank.empty():
                self.failsafe_time = curr_time
                msg = self.to_tank.get()
                if msg['t'] == "throttle":
                    self.r_t, self.l_t = msg['throttle']
                elif msg['t'] == "fire_weapon":
                    self.weapon_reload_time = curr_time
                    self.weapon_ready = False
                    self.weapon.send(ord('A'))
                elif msg['t'] == "shutdown":
                    self.running = False

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

                #right motor speed
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

            #send current status
            if not self.to_tornado.full() and time.time() * 1000 - self.heartbeat_time > 100:
                self.heartbeat_time = time.time() * 1000
                self.to_tornado.put({ 't': 'status', 'current': self.current, 'volts': self.volts, 'l_t': self.l_t, 'r_t': self.r_t, 'x': self.pos_x, 'y': self.pos_y, 'z': self.pos_z, 'a_x': self.azim_x, 'a_y': self.azim_y, 'a_z': self.azim_z, 'quality': self.pos_quality })
            time.sleep(0.01)

        print("Shutting down...")
        if serial_port_enabled:
            self.pi.serial_close(self.s1)


class WebsocketClient(Thread):
    def __init__(self):
        super(WebsocketClient, self).__init__()
        self.ioloop = tornado.ioloop.IOLoop.instance()
        self.ws = None
        self.connect()
        tornado.ioloop.PeriodicCallback(self.keep_alive, 1000).start()
        self.ioloop.start()

    @gen.coroutine
    def connect(self):
        print("trying to connect")
        try:
            self.ws = yield tornado.websocket.websocket_connect('ws://192.168.97.130:8000/ws', on_message_callback=self.on_message)
        except:
            time.sleep(2)
            self.connect()
        else:
            print("connected")
            self.send(json.dumps({ 't':'join', 'tid': ID }))

    def on_message(self, msg):
        if msg == None:
            self.ws.close()
            self.ws = None
            time.sleep(2)
            self.connect()
        else:
            print('here', msg)

    def send(self, msg):
        if not self.ws == None:
            self.ws.write_message(msg)
            return True
        return False

    def keep_alive(self):
        self.send(json.dumps({ 't': 'hb', 'tid': ID }))


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
        connections = set()

        def initialize(self, to_tornado, to_tank, to_main, running):
            self.to_tornado = to_tornado
            self.to_main = to_main
            self.to_tank = to_tank
            self.running = running
            tornado.ioloop.PeriodicCallback(self.update_sockets, 100).start()

        def update_sockets(self):
            if not self.running:
                self.close()

        def close(self):
            [client.close() for client in self.connections]
     
        def open(self):
            self.connections.add(self)
     
        def on_message(self, msg):
            msg = json.loads(msg)
            if msg['t'] == 't':
                r, l = steering2(msg['x'], msg['y'])
                self.to_tank.put({'t': 'throttle', 'throttle': (r,l)})
            elif msg['t'] == 'f':
                self.to_tank.put({'t': 'fire_weapon'})
            while not self.to_tornado.empty():
                msg = self.to_tornado.get()
                [client.write_message(json.dumps(msg)) for client in self.connections]

        def on_close(self):
            self.connections.remove(self)

    def signal_handler(self, signal, frame):
        print("Stopping Tornado.")
        self.running = False
        tornado.ioloop.IOLoop.current().stop()

    def run(self):
        self.running = True
        print('Starting webserver.')
        signal.signal(signal.SIGINT, self.signal_handler)

        public = os.path.join(os.path.dirname(__file__), 'tank_public')

        self.webapp = tornado.web.Application([
            (r"/tank_ws", self.WebSocket, {'to_tornado': self.to_tornado, 'to_main': self.to_main, 'to_tank': self.to_tank, 'running': self.running }),
            (r'/(.*)', self.MyStaticFileHandler, {'path': public, "default_filename": "index.html" }),
        ])

        self.webapp.listen(8000)
        tornado.ioloop.IOLoop.current().start()

def main():
    running = True

    set_start_method('spawn')

    to_tornado = Queue()
    to_tank = Queue()
    to_main = Queue()

    tank = Tank(to_tank, to_tornado, to_main)
    tank.start()

    web = Webserver(to_tornado, to_tank, to_main)
    web.start()

    cs = WebsocketClient()
    cs.start()

    while running == True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            running = False
            continue

    print('Shutting down.')

if __name__ == "__main__":
    main()
