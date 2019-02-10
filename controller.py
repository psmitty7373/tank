#!/usr/bin/python3

import BNO055, json, math, multiprocessing, pigpio, pfilter, os, signal, struct, time
from functools import partial
from statistics import median
import tornado.ioloop
import tornado.web as web
import tornado.websocket

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def steering(x, y, minJoystick, maxJoystick, minSpeed, maxSpeed):
	if x == 0 and y == 0:
		return (0, 0)

	z = math.sqrt(x * x + y * y)
	rad = math.acos(math.fabs(x) / z)
	angle = rad * 180 / math.pi
	tcoeff = -1 + (angle / 90) * 2
	turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
	turn = round(turn * 100, 0) / 100
	mov = max(math.fabs(y), math.fabs(x))

	if (x >= 0 and y >= 0) or (x < 0 and y < 0):
		rawLeft = mov
		rawRight = turn
	else:
		rawRight = mov
		rawLeft = turn

	if y < 0:
		rawLeft = 0 - rawLeft
		rawRight = 0 - rawRight

	rightOut = translate(rawRight, minJoystick, maxJoystick, minSpeed, maxSpeed)
	leftOut = translate(rawLeft, minJoystick, maxJoystick, minSpeed, maxSpeed)

	return (round(rightOut), round(leftOut))

right_pwm = 10
right_s1 = 27
right_s2 = 22

left_pwm = 17 
left_s1 = 9 
left_s2 = 11

serial_port_enabled = True
serial_port_mode = "UWB"
serial_port_baud = 115200
serial_port = "/dev/serial0"

class Tank(multiprocessing.Process):
    def __init__(self, tasks, results):
        super(Tank, self).__init__()

        self.running = True

        # init signal handler
        signal.signal(signal.SIGINT, self.shutdown)

        #init pigpio
        self.pi = pigpio.pi('localhost', 9999)

        if serial_port_enabled:
            self.s1 = self.pi.serial_open(serial_port, serial_port_baud)
        else:
            self.s1 = None

        #init imu
        self.bno = BNO055.BNO055()
        if self.bno.begin() is not True:
            print("Error initializing device")
            return False
        self.bno.setExternalCrystalUse(True)

        #right
        self.pi.set_mode(right_pwm, pigpio.OUTPUT)
        self.pi.set_mode(right_s1, pigpio.OUTPUT)
        self.pi.set_mode(right_s2, pigpio.OUTPUT)
        self.pi.write(right_s1, 0)
        self.pi.write(right_s2, 1)
        self.right_dir = 'f'
        self.right_braking = False
        self.right_brake_time = time.time() * 1000

        #left
        self.pi.set_mode(left_pwm, pigpio.OUTPUT)
        self.pi.set_mode(left_s1, pigpio.OUTPUT)
        self.pi.set_mode(left_s2, pigpio.OUTPUT)
        self.pi.write(left_s1, 1)
        self.pi.write(left_s2, 0)
        self.left_dir = 'f'
        self.left_braking = False
        self.left_brake_time = time.time() * 1000

        self.current = 0
        self.volts = 0

        # timers
        self.brake_time = 100
        self.failsafe_time = time.time() * 1000
        self.heartbeat_time = time.time() * 1000
        self.position_time = time.time() * 1000

        # position
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
        self.tasks = tasks
        self.results = results

        #particle filter
        self.pf = pfilter.ParticleFilter(200, 0.1, (10, 10, 10))

    def shutdown(self, signal, frame):
        self.running = False

    def update_uwb(self):
        rlv_type = 0
        rlv_payload = b""
        num_bytes, rx_bytes = self.pi.serial_read(self.s1, 2)
        if num_bytes == 2:
            rlv_type, rlv_payload_len = struct.unpack('<bb', rx_bytes)
            if rlv_payload_len > 0:
                num_bytes, rlv_payload = self.pi.serial_read(self.s1, rlv_payload_len)
        if not rlv_type == 0:
            self.process_rlv_pkt({ 'rlv_type': rlv_type, 'rlv_payload': rlv_payload })

    def process_rlv_pkt(self, pkt):
        if pkt['rlv_type'] == 0:
            return
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
#        else:
#            print("Got:", pkt)

    def run(self):
        print("Running!")
        while self.running:
            curr_time = time.time() * 1000
            # if no update in a second, stop motors
            if curr_time - self.failsafe_time > 1000:
                self.pi.set_servo_pulsewidth(left_pwm, 0)
                self.pi.set_servo_pulsewidth(right_pwm, 0)

            # get position info every 100ms
            if curr_time - self.position_time > 100:
                self.position_time = curr_time
#                self.pi.serial_write(self.s1, '\x02\x00') #get curr position
                self.pi.serial_write(self.s1, '\x0c\x00') #get measurements
                self.pf.update()
                self.ppos_x, self.ppos_y, self.ppos_z = self.pf.getEstimate()
                self.azim_x, self.azim_y, self.azim_z = self.bno.getVector(BNO055.BNO055.VECTOR_EULER)

            self.update_uwb()

            while not self.tasks.empty():
                self.failsafe_time = curr_time
                task = self.tasks.get()
                if task['task_type'] == "throttle_update":
                    r, l = task['payload']
                elif task['task_type'] == "shutdown":
                    self.running = False

                # right motor
                if r > 0: # forward
                    # changing directions
                    if self.right_dir == 'b':
                        # start braking
                        if not self.right_braking:
                            self.right_braking = True
                            self.right_brake_time = time.time() * 1000
                            self.pi.write(right_s1, 0)
                            self.pi.write(right_s2, 0)

                        # done braking
                        if time.time() * 1000 - self.right_brake_time > self.brake_time:
                            self.right_braking = False
                            self.pi.write(right_s1, 0)
                            self.pi.write(right_s2, 1)
                            self.right_dir = 'f'

                    # same dir, cancel braking
                    elif self.right_braking:
                        self.right_braking = False
                        self.pi.write(right_s1, 0)
                        self.pi.write(right_s2, 1)

                elif r < 0:
                    if self.right_dir == 'f':
                        # start braking
                        if not self.right_braking:
                            self.right_braking = True
                            self.right_brake_time = time.time() * 1000
                            self.pi.write(right_s1, 0)
                            self.pi.write(right_s2, 0)

                        # done braking
                        if time.time() * 1000 - self.right_brake_time > self.brake_time:
                            self.right_braking = False
                            self.pi.write(right_s1, 1)
                            self.pi.write(right_s2, 0)
                            self.right_dir = 'b'

                    # same dir, cancel braking
                    elif self.right_braking:
                        self.right_braking = False
                        self.pi.write(right_s1, 1)
                        self.pi.write(right_s2, 0)

                # left motor
                if l > 0: # forward
                    # changing directions
                    if self.left_dir == 'b':
                        # start braking
                        if not self.left_braking:
                            self.left_braking = True
                            self.left_brake_time = time.time() * 1000
                            self.pi.write(left_s1, 0)
                            self.pi.write(left_s2, 0)

                        # done braking
                        if time.time() * 1000 - self.left_brake_time > self.brake_time:
                            self.left_braking = False
                            self.pi.write(left_s1, 1)
                            self.pi.write(left_s2, 0)
                            self.left_dir = 'f'

                    # same dir, cancel braking
                    elif self.left_braking:
                        self.left_braking = False
                        self.pi.write(left_s1, 1)
                        self.pi.write(left_s2, 0)

                elif l < 0:
                    if self.left_dir == 'f':
                        # start braking
                        if not self.left_braking:
                            self.left_braking = True
                            self.left_brake_time = time.time() * 1000
                            self.pi.write(left_s1, 0)
                            self.pi.write(left_s2, 0)

                        # done braking
                        if time.time() * 1000 - self.left_brake_time > self.brake_time:
                            self.left_braking = False
                            self.pi.write(left_s1, 0)
                            self.pi.write(left_s2, 1)
                            self.left_dir = 'b'

                    # same dir, cancel braking
                    elif self.left_braking:
                        self.left_braking = False
                        self.pi.write(left_s1, 0)
                        self.pi.write(left_s2, 1)

                if r == 0 or self.right_braking:
                    self.pi.set_servo_pulsewidth(left_pwm, 0)
                else:
                    self.pi.set_servo_pulsewidth(left_pwm, translate(abs(r), 0, 100, 500, 2000))

                if l == 0 or self.left_braking:
                    self.pi.set_servo_pulsewidth(right_pwm, 0)
                else:
                    self.pi.set_servo_pulsewidth(right_pwm, translate(abs(l), 0, 100, 500, 2000))

                #semd status back
                if not self.results.full() and time.time() * 1000 - self.heartbeat_time > 200:
                    self.heartbeat_time = time.time() * 1000
                    self.results.put({ 'current': self.current, 'volts': self.volts, 'x': median(self.posm_x)/10.0, 'y': median(self.posm_y)/10.0, 'z': median(self.posm_z)/10.0, 'a_x': self.azim_x, 'a_y': self.azim_y, 'a_z': self.azim_z, 'quality': self.pos_quality })
            time.sleep(0.01)

        print("Shutting down...")
        self.pi.serial_close(self.s1)


public = os.path.join(os.path.dirname(__file__), 'public')

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html")

class MyStaticFileHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class WebSocket(tornado.websocket.WebSocketHandler):
    connections = set()

    def initialize(self, tasks, results):
        self.tasks = tasks
        self.results = results
 
    def open(self):
        self.connections.add(self)
 
    def on_message(self, message):
        pos = json.loads(message)
        r, l = steering(pos['x'], pos['y'], -100, 100, -100, 100)
        self.tasks.put({'task_type': 'throttle_update', 'payload': (r,l)})
        while not self.results.empty():
            msg = self.results.get()
            [client.write_message(json.dumps(msg)) for client in self.connections]

    def on_close(self):
        self.connections.remove(self)
 
def make_app(tasks, results):
    return tornado.web.Application([
        (r"/tank_ws", WebSocket, {'tasks': tasks, 'results': results}),
        (r'/(.*)', MyStaticFileHandler, {'path': public, "default_filename": "index.html"}),
    ])


def signal_handler(signal, frame):
    tornado.ioloop.IOLoop.current().stop()

def main():
    tasks = multiprocessing.JoinableQueue()
    results = multiprocessing.Queue(maxsize=2)
    tank = Tank(tasks, results)
    tank.start()

    webapp = make_app(tasks, results)
    webapp.listen(8000)
    signal.signal(signal.SIGINT, signal_handler)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
