#!/usr/bin/python3

import json, math, multiprocessing, os, struct, time
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

        import pigpio
        self.pi = pigpio.pi('localhost', 9999)

        if serial_port_enabled:
            self.s1 = self.pi.serial_open(serial_port, serial_port_baud)
        else:
            self.s1 = None

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
        self.brake_time = 100#ms
        self.failsafe_time = time.time() * 1000
        self.heartbeat_time = time.time() * 1000
        self.position_time = time.time() * 1000
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.pos_quality = 0
        self.tasks = tasks
        self.results = results

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
        if pkt['rlv_type'] == 65 and len(pkt['rlv_payload']) == 13:
            self.pos_x, self.pos_y, self.pos_z, self.pos_quality = struct.unpack('<lllb', pkt['rlv_payload'])

    def run(self):
        print("Running!")
        while True:
            curr_time = time.time() * 1000
            # if no update in a second, stop motors
            if curr_time - self.failsafe_time > 1000:
                self.pi.set_servo_pulsewidth(left_pwm, 0)
                self.pi.set_servo_pulsewidth(right_pwm, 0)

            if curr_time - self.position_time > 200:
                self.pi.serial_write(self.s1, '\x02\x00')

            self.update_uwb()

            while not self.tasks.empty():
                self.failsafe_time = curr_time
                r, l = self.tasks.get()

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
                    self.results.put({ 'current': self.current, 'volts': self.volts, 'x': self.pos_x, 'y': self.pos_y, 'z': self.pos_z, 'quality': self.pos_quality })
            time.sleep(0.01)


public = os.path.join(os.path.dirname(__file__), 'public')
 
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html")

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
        self.tasks.put((r,l))
        while not self.results.empty():
            msg = self.results.get()
            [client.write_message(json.dumps(msg)) for client in self.connections]

    def on_close(self):
        self.connections.remove(self)
 
def make_app(tasks, results):
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/tank_ws", WebSocket, {'tasks': tasks, 'results': results}),
        (r'/(.*)', web.StaticFileHandler, {'path': public}),
    ])

if __name__ == "__main__":
    tasks = multiprocessing.JoinableQueue()
    results = multiprocessing.Queue(maxsize=2)
    tank = Tank(tasks, results)
    tank.start()

    webapp = make_app(tasks, results)
    webapp.listen(8000)
    tornado.ioloop.IOLoop.current().start()
