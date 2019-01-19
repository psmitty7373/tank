#!/usr/bin/python3

import json, math, multiprocessing, os, time
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

class Tank(multiprocessing.Process):
    def __init__(self, task):
        super(Tank, self).__init__()
        import pigpio
        self.pi = pigpio.pi('localhost', 9999)
        self.pi.set_mode(17, pigpio.OUTPUT)
        self.pi.set_mode(27, pigpio.OUTPUT)
        self.pi.set_mode(22, pigpio.OUTPUT)
        self.pi.write(27, 0)
        self.pi.write(22, 1)

        #left
        self.pi.set_mode(10, pigpio.OUTPUT)
        self.pi.set_mode(9, pigpio.OUTPUT)
        self.pi.set_mode(11, pigpio.OUTPUT)
        self.pi.write(9, 1)
        self.pi.write(11, 0)

        self.left_dir = 'f'
        self.right_dir = 'f'
        self.brake_time = 100#ms
        self.left_braking = False
        self.right_braking = False
        self.left_brake_time = time.time() * 1000
        self.right_brake_time = time.time() * 1000

        self.tasks = tasks
        print("Tank Init Complete!")
     
    def run(self):
        print("Running!")
        while True:
            if not self.tasks.empty():
                r, l = self.tasks.get()

                # right motor
                if r > 0: # forward
                    # changing directions
                    if self.right_dir == 'b':
                        # start braking
                        if not self.right_braking:
                            print("brake_right")
                            self.right_braking = True
                            self.right_brake_time = time.time() * 1000
                            self.pi.write(27, 0)
                            self.pi.write(22, 0)

                        # done braking
                        if time.time() * 1000 - self.right_brake_time > self.brake_time:
                            print("done_brake_right")
                            self.right_braking = False
                            self.pi.write(27, 0)
                            self.pi.write(22, 1)
                            self.right_dir = 'f'

                    # same dir, cancel braking
                    elif self.right_braking:
                        self.right_braking = False
                        self.pi.write(27, 0)
                        self.pi.write(22, 1)

                elif r < 0:
                    if self.right_dir == 'f':
                        # start braking
                        if not self.right_braking:
                            print("brake_right")
                            self.right_braking = True
                            self.right_brake_time = time.time() * 1000
                            self.pi.write(27, 0)
                            self.pi.write(22, 0)

                        # done braking
                        if time.time() * 1000 - self.right_brake_time > self.brake_time:
                            print("done_brake_right")
                            self.right_braking = False
                            self.pi.write(27, 1)
                            self.pi.write(22, 0)
                            self.right_dir = 'b'

                    # same dir, cancel braking
                    elif self.right_braking:
                        self.right_braking = False
                        self.pi.write(27, 1)
                        self.pi.write(22, 0)

                # left motor
                if l > 0: # forward
                    # changing directions
                    if self.left_dir == 'b':
                        # start braking
                        if not self.left_braking:
                            print("brake_left")
                            self.left_braking = True
                            self.left_brake_time = time.time() * 1000
                            self.pi.write(9, 0)
                            self.pi.write(11, 0)

                        # done braking
                        if time.time() * 1000 - self.left_brake_time > self.brake_time:
                            print("done_brake_left")
                            self.left_braking = False
                            self.pi.write(9, 1)
                            self.pi.write(11, 0)
                            self.left_dir = 'f'

                    # same dir, cancel braking
                    elif self.left_braking:
                        self.left_braking = False
                        self.pi.write(9, 1)
                        self.pi.write(11, 0)

                elif l < 0:
                    print(self.left_dir)
                    if self.left_dir == 'f':
                        # start braking
                        if not self.left_braking:
                            print("brake_left")
                            self.left_braking = True
                            self.left_brake_time = time.time() * 1000
                            self.pi.write(9, 0)
                            self.pi.write(11, 0)

                        # done braking
                        if time.time() * 1000 - self.left_brake_time > self.brake_time:
                            print("done_brake_left")
                            self.left_braking = False
                            self.pi.write(9, 0)
                            self.pi.write(11, 1)
                            self.left_dir = 'b'

                    # same dir, cancel braking
                    elif self.left_braking:
                        self.left_braking = False
                        self.pi.write(9, 0)
                        self.pi.write(11, 1)

                if r == 0 or self.right_braking:
                    self.pi.set_servo_pulsewidth(10, 0)
                else:
                    self.pi.set_servo_pulsewidth(10, translate(abs(r), 0, 100, 500, 2500))

                if l == 0 or self.left_braking:
                    self.pi.set_servo_pulsewidth(17, 0)
                else:
                    self.pi.set_servo_pulsewidth(17, translate(abs(l), 0, 100, 500, 2500))
            time.sleep(0.01)


public = os.path.join(os.path.dirname(__file__), 'public')
 
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html")

class WebSocket(tornado.websocket.WebSocketHandler):
    connections = set()

    def initialize(self, tasks):
        self.tasks = tasks
 
    def open(self):
        self.connections.add(self)
 
    def on_message(self, message):
        pos = json.loads(message)
        r, l = steering(pos['x'], pos['y'], -100, 100, -100, 100)
        print(pos, l, r)
        self.tasks.put((r,l))

    def on_close(self):
        self.connections.remove(self)
 
def make_app(tasks):
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/tank_ws", WebSocket, {'tasks': tasks}),
        (r'/(.*)', web.StaticFileHandler, {'path': public}),
    ])

if __name__ == "__main__":
    tasks = multiprocessing.JoinableQueue()
    tank = Tank(tasks)
    tank.start()

    webapp = make_app(tasks)
    webapp.listen(8000)
    tornado.ioloop.IOLoop.current().start()
