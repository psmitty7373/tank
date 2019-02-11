#!/usr/bin/python3

import datetime, json, math, multiprocessing, pigpio, os, signal, struct, time
from pykalman import KalmanFilter
import numpy as np
from statistics import median
import tornado.ioloop
import tornado.web as web
import tornado.websocket

serial_port_enabled = True
serial_port_mode = "UWB"
serial_port_baud = 115200
serial_port = "/dev/serial0"

transition_matrix = [[1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]]

observation_matrix = [[1, 0, 0, 0],
                      [0, 0, 1, 0]]

class UWB(multiprocessing.Process):
    def __init__(self, tasks, results):
        super(UWB, self).__init__()

        self.running = True

        # init signal handler
        signal.signal(signal.SIGINT, self.shutdown)

        #init pigpio
        self.pi = pigpio.pi('localhost')

        if serial_port_enabled:
            self.s1 = self.pi.serial_open(serial_port, serial_port_baud)
        else:
            self.s1 = None

        #timers
        self.heartbeat_time = time.time() * 1000
        self.position_time = time.time() * 1000

        # messaging
        self.tasks = tasks
        self.results = results
        self.tanks = {}

    def shutdown(self, signal, frame):
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

            #self.results.put({ 'id': tank_id, 'x': median(self.tanks[tank_id]['posm_x']), 'y': median(self.tanks[tank_id]['posm_y']), 'z': median(self.tanks[tank_id]['posm_z']) })
            self.results.put({ 'id': tank_id, 'x': x, 'y': y, 'z': median(self.tanks[tank_id]['posm_z']) })

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
        print("Running!")
        self.enable_shell_mode()
        while self.running:
            curr_time = time.time() * 1000
            self.shell_recv()

            # get position info every 100ms
            if curr_time - self.position_time > 100:
                self.position_time = curr_time

            while not self.tasks.empty():
                self.failsafe_time = curr_time
                task = self.tasks.get()
                if task['task_type'] == "shutdown":
                    self.running = False

                #semd status back
                if not self.results.full() and time.time() * 1000 - self.heartbeat_time > 200:
                    self.heartbeat_time = time.time() * 1000
            time.sleep(0.1)

        print("Shutting down...")
        self.pi.serial_close(self.s1)

public = os.path.join(os.path.dirname(__file__), 'server')

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
        tornado.ioloop.PeriodicCallback(self.flush_queue, 100).start()

    def flush_queue(self):
        while not self.results.empty():
            msg = self.results.get()
            [client.write_message(json.dumps(msg)) for client in self.connections]
 
    def open(self):
        self.connections.add(self)
 
    def on_message(self, message):
        pos = json.loads(message)

    def on_close(self):
        self.connections.remove(self)
 
def make_app(tasks, results):
    return tornado.web.Application([
        (r"/ws", WebSocket, {'tasks': tasks, 'results': results}),
        (r'/(.*)', MyStaticFileHandler, {'path': public, "default_filename": "index.html"}),
    ])


def signal_handler(signal, frame):
    tornado.ioloop.IOLoop.current().stop()

def main():
    tasks = multiprocessing.JoinableQueue()
    results = multiprocessing.Queue(maxsize=64)
    uwb = UWB(tasks, results)
    uwb.start()

    webapp = make_app(tasks, results)
    webapp.listen(8000)
    signal.signal(signal.SIGINT, signal_handler)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
