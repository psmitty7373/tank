#!/usr/bin/python3

import base64, cv2, datetime, imutils, json, math, numpy as np, os, pickle, pigpio, random, re, signal, struct, time
from multiprocessing import Process, Manager, Queue, set_start_method
from multiprocessing.managers import BaseManager
from imutils import contours
from pykalman import KalmanFilter
from statistics import median
import tornado.ioloop
import tornado.web as web
import tornado.websocket

serial_port_enabled = True
serial_port_mode = "UWB"
serial_port_baud = 115200
serial_port = "/dev/serial0"

camera_enabled = True

transition_matrix = [[1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]]

observation_matrix = [[1, 0, 0, 0],
                      [0, 0, 1, 0]]

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

class Game(object):
    def __init__(self):
        self.tanks = {}

    def add_tank(self, x=0, y=0, tid=0):
        if tid in self.tanks.keys():
            return
        self.tanks[tid] = { 'pos': [x ,y], 'last_update': time.time(), 'last_poll': time.time() }

    def touch_tank(self, tid):
        if tid in self.tanks.keys():
            self.tanks[tid]['last_update'] = time.time()

    def cleanup_tanks(self):
        now = time.time()
        for t in list(self.tanks.keys()):
            if now - self.tanks[t]['last_update'] > 30:
                print('removing tank', t)
                self.tanks.pop(t, None)

    def update_tank_pos(self, tid, pos):
        if tid in self.tanks.keys():
            self.tanks[tid]['pos'] = pos

    def get_oldest_poll_tid(self):
        old_time = time.time() - 3
        old_tank = None
        for t in list(self.tanks.keys()):
            if self.tanks[t]['last_poll'] < old_time:
                old_tank = t
                old_time = self.tanks[t]['last_poll']
        if not old_tank == None:
            print('setting old_tank', old_tank)
            self.tanks[old_tank]['last_poll'] = time.time()
        return old_tank

    def get_tanks(self):
        return json.dumps(self.tanks)

    def find_tank(self, pos):
        closest_dist = None
        closest_tank = None
        for t in self.tanks:
            dist = np.linalg.norm(np.array(pos) - np.array(t.pos))
            if closest_tank == None or dist < closest_dist:
                closest_dist = dist
                closest_tank = t
        if closest_dist < 20.0:
            return closest_tank
        else:
            return False

class Location_Cam(Process):
    def __init__(self, to_cam, to_main, g):
        super(Location_Cam, self).__init__()
        self.to_cam = to_cam
        self.to_main = to_main
        self.g = g

    def update_bitstreams(self):
        for b in self.bitstreams:
            if len(b['stream']) == 50 and (sum(b['stream']) == 0 or b['bad_count'] > 256):
                self.bitstreams.remove(b)
                continue

            b['stream'].append(0)
            b['poll_count'] += 1
            b['stream'] = b['stream'][-50:]

    def read_bitstreams(self):
        for b in self.bitstreams:
            if len(b['stream']) < 50:
                continue

            chr_stream = ''.join(str(b) for b in b['stream'])            
            chr_stream = chr_stream[chr_stream.find('01')+1:]
            chr_stream = chr_stream[:40]
            pulse_count = re.sub('00+', '0', re.sub('11+', '1', chr_stream)).count('1')
            if '1' in chr_stream:
                pulse_width = float(chr_stream.count('1')) / pulse_count
            else:
                pulse_width = 0

            print(b['tid'], chr_stream, b['confidence'], pulse_count, pulse_width)

            if pulse_count == 4 and pulse_width >= 6:
                if b['confidence'] < 1:
                    b['confidence'] = 1

            elif not self.polling and pulse_count < 3:
                b['confidence'] = 0
                b['bad_count'] += 1

            if b['confidence'] > 0 and self.polling and pulse_count == 4 and pulse_width < 6:
                if b['tid'] != self.polling:
                    print('tank mismatch!!')
                else:
                    b['good_count'] += 1
                print('found tank, probably...')
                b['confidence'] = 2
                b['tid'] = self.polling
                self.polling = False


    def find_and_set_bitstream(self, pos):
        closest_bs = None
        closest_dist = None

        for b in self.bitstreams:
            dist = np.linalg.norm(np.array(pos) - np.array(b['pos']))
            if closest_bs == None or dist < closest_dist:
                closest_dist = dist
                closest_bs = b

        if not closest_bs == None:
            if closest_dist < 50.0:
                closest_bs['pos'] = pos
                closest_bs['stream'][-1] = 1
                closest_bs['seen_count'] += 1
                if closest_bs['tid'] > -1:
                    tank_pos = [ translate(pos[0], self.calibration_data['tl'][0], self.calibration_data['tr'][0], 0, 200),
                            translate(pos[1], self.calibration_data['tl'][1], self.calibration_data['bl'][1], 0, 200) ]
                    self.g.update_tank_pos(closest_bs['tid'], tank_pos)
                    self.to_main.put({ 't': 'pos', 'tid': closest_bs['tid'], 'pos': tank_pos })
                return
            else:
                print('huge jump:', closest_dist)

        self.bitstreams.append({'pos': pos, 'stream': [0], 'seen_count': 1, 'poll_count': 1, 'tid': -1, 'confidence': 0, 'bad_count': 0, 'good_count': 0})

    def shutdown(self, signal, frame):
        print("Stopping camera.")
        self.running = False

    def get_cam(self):
        ret, image = self.cam.read()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
        self.to_main.put({'t': 'cam', 'image': base64.b64encode(cv2.imencode('.png', thresh)[1].tostring()).decode('ascii')})

    def calibrate_arena(self, corners):
        self.calibration_data = corners
        with open('arena.conf', 'wb') as f:
            pickle.dump(self.calibration_data, f)

    def track_tanks(self):
        self.update_bitstreams()
        ret, image = self.cam.read()
        now = time.time()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 215, 255, cv2.THRESH_BINARY)[1]
#        thresh = cv2.erode(thresh, None, iterations=2)
#        thresh = cv2.dilate(thresh, None, iterations=4)
        cnts = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        found = False
        if len(cnts) > 0:
            for (i, c) in enumerate(cnts):
                found = True
                ((cX, cY), radius) = cv2.minEnclosingCircle(c)
                if radius > 1.0 and radius < 20.0 and cX >= self.calibration_data['tl'][0] and cX <= self.calibration_data['tr'][0] and cY >= self.calibration_data['tl'][1] and cY <= self.calibration_data['bl'][1]:
#                    cv2.circle(image, (int(cX), int(cY)), int(radius), (0, 0, 255), 1)
#                    cv2.putText(image, "#{}".format(i + 1), (int(cX), int(cY) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
                    self.find_and_set_bitstream([cX, cY])

        self.tot_time += now - self.last_frame
        self.frames += 1
        self.last_frame = now

        if now - self.last_show > 5:
#            cv2.imshow("hsv", thresh)
            print(1 / (self.tot_time / self.frames))
            #print(self.bitstreams)
            self.last_show = now

        now = time.time()
        if now - self.last_bitstream_read > 0.25:
            self.read_bitstreams()
            self.last_bitstream_read = now

#        if cv2.waitKey(1) == 27:
#            return

    def run(self):
        self.running = True

        # init signal handler
        signal.signal(signal.SIGINT, self.shutdown)

        # setup camera
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam.set(cv2.CAP_PROP_FPS, 40)
        time.sleep(1)

        self.polling = False
        self.bitstreams = []

        self.tot_time = 0
        self.frames = 0
        self.last_frame = time.time()
        self.last_show = time.time()
        self.last_bitstream_read = time.time()

        if os.path.isfile('arena.conf'):
            print("Loaded arena.conf.")
            with open('arena.conf','rb') as f:
                cd = pickle.load(f)
                self.calibration_data = cd
        else:
            self.calibration_data = {'tl': [0,0], 'tr': [640,0], 'bl': [0, 480], 'br': [640, 480]}

        print("Camera started.")
        #cv2.namedWindow('hsv')
        poll_time = time.time()
        while self.running:
            now = time.time()
            if self.polling and now - poll_time > 5:
                print('not found')
                self.polling = False

            if not self.polling and now - poll_time > 2:
                self.g.cleanup_tanks()
                poll_time = now
                old_tid = self.g.get_oldest_poll_tid()
                print('Polling for:', old_tid)
                if not old_tid == None:
                    self.polling = old_tid
                    self.to_main.put({'t': 'poll', 'tid': old_tid})

            self.track_tanks()

            while not self.to_cam.empty():
                msg = self.to_cam.get()
                if msg['t'] == 'get_cam':
                    self.get_cam()
                elif msg['t'] == 'calibrate':
                    self.calibrate_arena(msg['corners'])

        cv2.destroyAllWindows()

class UWB(Process):
    def __init__(self, to_uwb, to_main):
        super(UWB, self).__init__()
        self.to_uwb = to_uwb
        self.to_main = to_main

    def shutdown(self, signal, frame):
        print('Stopping UWB.')
        self.running = False

    def enable_shell_mode(self):
        if self.s1 == None:
            return
        print ('Enabling shell!')
        self.pi.serial_write(self.s1, 'reset\r')
        time.sleep(5)
        self.pi.serial_write(self.s1, '\r')
        time.sleep(0.1)
        self.pi.serial_write(self.s1, '\r')
        time.sleep(2)
        self.pi.serial_write(self.s1, "lep\r")

    def process_shell_pkt(self, pkt):
        if len(pkt) == 0:
            return
        pkt = pkt.split(b',')
        if pkt[0] == b'POS' and len(pkt) == 8:
            tank_id = bytearray(pkt[2]).decode('utf-8')
            x = float(bytearray(pkt[3]).decode('utf-8'))*100
            y = float(bytearray(pkt[4]).decode('utf-8'))*100
            z = float(bytearray(pkt[5]).decode('utf-8'))*100

            if tank_id not in self.tanks.keys():
                self.tanks[tank_id] = { 'x': 0, 'y': 0, 'z': 0, 'posm_x': [], 'posm_y': [], 'posm_z': [], 'pos_history': [], 'kf': None }

            self.tanks[tank_id]['x'] = x
            self.tanks[tank_id]['y'] = y
            self.tanks[tank_id]['z'] = z

            if len(self.tanks[tank_id]['pos_history']) < 20:
                self.tanks[tank_id]['pos_history'].append((x,y))
            elif self.tanks[tank_id]['kf'] == None:
                print('building kf')
                measurements = np.asarray(self.tanks[tank_id]['pos_history'])
                kf = KalmanFilter(transition_matrices = transition_matrix,
                        observation_matrices = observation_matrix,
                        initial_state_mean = [measurements[0, 0], 0, measurements[0, 1], 0])
                kf = kf.em(measurements, n_iter=5)
                self.tanks[tank_id]['kf'] = KalmanFilter(transition_matrices = transition_matrix,
                        observation_matrices = observation_matrix,
                        initial_state_mean = [measurements[0, 0], 0, measurements[0, 1], 0],
                        observation_covariance = 10*kf.observation_covariance,
                        em_vars=['transition_covariance', 'initial_state_covariance'])
                self.tanks[tank_id]['kf'] = self.tanks[tank_id]['kf'].em(measurements, n_iter=5)
                self.tanks[tank_id]['means'], self.tanks[tank_id]['covariances'] = self.tanks[tank_id]['kf'].filter(measurements)
                self.tanks[tank_id]['means'] = self.tanks[tank_id]['means'][-1, :]
                self.tanks[tank_id]['covariances'] =  self.tanks[tank_id]['covariances'][-1, :]
            else:
                self.tanks[tank_id]['means'], self.tanks[tank_id]['covariances'] = self.tanks[tank_id]['kf'].filter_update(filtered_state_mean = self.tanks[tank_id]['means'],
                        filtered_state_covariance = self.tanks[tank_id]['covariances'], observation = (x,y))
                x = self.tanks[tank_id]['means'][0]
                y = self.tanks[tank_id]['means'][2]

            self.tanks[tank_id]['posm_x'].append(x)
            self.tanks[tank_id]['posm_y'].append(y)
            self.tanks[tank_id]['posm_z'].append(z)

            if len(self.tanks[tank_id]['posm_x']) > 5:
                self.tanks[tank_id]['posm_x'] = self.tanks[tank_id]['posm_x'][-5:]
                self.tanks[tank_id]['posm_y'] = self.tanks[tank_id]['posm_y'][-5:]
                self.tanks[tank_id]['posm_z'] = self.tanks[tank_id]['posm_z'][-5:]

            #self.to_main.put({ 'id': tank_id, 'x': median(self.tanks[tank_id]['posm_x']), 'y': median(self.tanks[tank_id]['posm_y']), 'z': median(self.tanks[tank_id]['posm_z']) })
            self.to_main.put({ 't': 'pos', 'id': tank_id, 'x': x, 'y': y, 'z': median(self.tanks[tank_id]['posm_z']) })

    def shell_recv(self):
        if self.s1 == None:
            return
        num_bytes = 1
        buf = bytearray()
        while num_bytes > 0:
            num_bytes, rx_bytes = self.pi.serial_read(self.s1, 256)
            buf.extend(rx_bytes)
        if len(buf) > 0:
            for pkt in buf.split(b'\r\n'):
                self.process_shell_pkt(pkt)

    def run(self):
        self.running = True
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.heartbeat_time = time.time() * 1000
        self.position_time = time.time() * 1000

        self.tanks = {}

        self.pi = pigpio.pi('localhost')

        if serial_port_enabled and serial_port_mode == "UWB":
            self.s1 = self.pi.serial_open(serial_port, serial_port_baud)
        else:
            self.s1 = None

        self.enable_shell_mode()
        print("Running!")
        while self.running:
            curr_time = time.time() * 1000
            self.shell_recv()

            # get position info every 100ms
            if curr_time - self.position_time > 100:
                self.position_time = curr_time

            while not self.to_uwb.empty():
                self.failsafe_time = curr_time
                msg = self.to_uwb.get()
                if msg['t'] == "shutdown":
                    self.running = False

                #semd status back
                if not self.to_main.full() and time.time() * 1000 - self.heartbeat_time > 200:
                    self.heartbeat_time = time.time() * 1000
            time.sleep(0.01)

        print("Stopping UWB.")
        self.pi.serial_close(self.s1)

class Webserver(Process):
    def __init__(self, to_tornado, to_main, g):
        super(Webserver, self).__init__()
        self.to_tornado = to_tornado
        self.to_main = to_main
        self.g = g

    class MainHandler(tornado.web.RequestHandler):
        def get(self):
            self.render("index.html")

    class MyStaticFileHandler(tornado.web.StaticFileHandler):
        def set_extra_headers(self, path):
            self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

    class WebSocket(tornado.websocket.WebSocketHandler):

        def initialize(self, to_tornado, to_main, g, connections):
            self.to_tornado = to_tornado
            self.to_main = to_main
            self.g = g
            self.connections = connections

        def open(self):
            self.type = 'unk'
            self.tid = -1
            self.connections.add(self)
            print('connect')
     
        def on_message(self, message):
            msg = json.loads(message)

            if 't' not in msg.keys():
                return

            if msg['t'] == 'join':
                print('adding tank', msg['tid'])
                self.tid = msg['tid']
                self.type = 'tank'
                self.g.add_tank(tid=msg['tid'], x=0, y=0)

            if msg['t'] == 'server':
                self.type = 'server'

            elif msg['t'] == 'hb' and 'tid' in msg.keys():
                self.g.touch_tank(msg['tid'])

            else:
                self.to_main.put(msg)

        def on_close(self):
            self.connections.remove(self)

    def update_sockets(self):
        if time.time() - self.last_update > 0.1:
            self.last_update = time.time()
            tanks = self.g.get_tanks()
            for client in self.connections:
                if client.type == 'server' or client.type == 'unk':
                    client.write_message(json.dumps({'t': 'update', 'tanks': tanks}))

        while not self.to_tornado.empty():
            msg = self.to_tornado.get()
            for client in self.connections:
                if 'tid' in msg.keys() and client.tid == msg['tid']:
                    client.write_message(json.dumps(msg))
                elif 'tid' not in msg.keys() and client.type == 'server':
                    client.write_message(json.dumps(msg))

    def signal_handler(self, signal, frame):
        print('Stopping Webserver.')
        tornado.ioloop.IOLoop.current().stop()

    def run(self):
        signal.signal(signal.SIGINT, self.signal_handler)

        public = os.path.join(os.path.dirname(__file__), 'server_public')

        self.last_update = time.time()
        self.connections = set()
        tornado.ioloop.PeriodicCallback(self.update_sockets, 50).start()

        print('Starting Webserver.')
        self.webapp = tornado.web.Application([
            (r"/ws", self.WebSocket, {'to_tornado': self.to_tornado, 'to_main': self.to_main, 'g': self.g, 'connections': self.connections }),
            (r'/(.*)', self.MyStaticFileHandler, {'path': public, "default_filename": "index.html"}),
        ])
        self.webapp.listen(8000)
        tornado.ioloop.IOLoop.current().start()


def main():
    running = True

    set_start_method("spawn")

    BaseManager.register('Game', Game)
    manager = BaseManager()
    manager.start()
    g = manager.Game()

    to_uwb = Queue()
    to_cam = Queue()
    to_tornado = Queue()
    to_main = Queue(maxsize=64)

    #start uwb
    #if serial_port_enabled and serial_port_mode == "UWB":
    #    uwb = UWB(to_uwb, to_main)
    #    uwb.start()

    #start camera
    if camera_enabled:
        cam = Location_Cam(to_cam, to_main, g)
        cam.start()

    #start Webserver
    web = Webserver(to_tornado, to_main, g)
    web.start()

    print('Starting main loop.')
    while running:
        try:
            while not to_main.empty():
                msg = to_main.get()
                if msg['t'] == 'poll' or msg['t'] == 'pos':
                    to_tornado.put(msg)
                elif msg['t'] == 'get_cam' or msg['t'] == 'calibrate':
                    to_cam.put(msg)
                elif msg['t'] == 'cam':
                    to_tornado.put(msg)
            time.sleep(0.01)

        except KeyboardInterrupt:
            running = False
            continue

    print('Shutdown.')

if __name__ == "__main__":
    main()
