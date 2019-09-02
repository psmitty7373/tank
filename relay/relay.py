#!/usr/bin/python3

import argparse, select, signal, socket, sys, struct, time
from multiprocessing import Pipe
from threading import Thread

server_ip = '127.0.0.1'
server_port = 9999
running = True

OPEN = 0
CLOSE = 1
DATA = 2
READY = 3
NOTREADY = 4
ERROR = 5

class CustomSocket(socket.socket):
    def __init__(self, *args, **kwargs):
        super(CustomSocket, self).__init__(*args, **kwargs)
        self.lport = 0
        self.eport = 0
        self.sport = 0

def signal_handler(signal, frame):
    global running
    running = False

def recv_all(s, data_len):
    data = b''

    while len(data) < data_len:
        t_data = s.recv(data_len - len(data))

        if t_data == b'':
            return b''

        data += t_data

    return data

class Client(Thread):
    def __init__(self, ip, port):
        super(Client, self).__init__()
        self.server_port = port
        self.server_ip = ip
        self.socks = {}
        self.master_sock = None
        self.running = True

    def connect(self, ip, port, eport=0, master=False):
        try:
            s = CustomSocket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((ip, port))
            s.setblocking(0)

            if master:
                self.socks[0] = s
                self.master_sock = s
            else:
                self.socks[eport] = s

            s.sport = port
            s.eport = eport
            s.lport = s.getsockname()[1]

            print ('Opened:', s.eport, s.sport, s.lport)

            return True

        except:
            print('Error connecting to server.')
            return False

    def kill(self):
        for s in self.socks.values():
            s.close()
        self.socks = {}

    def run(self):
        while running:
            if 0 not in self.socks.keys():
                print('Connecting...')
                if not self.connect(self.server_ip, self.server_port, master=True):
                    time.sleep(5)
                    continue
            try:
                ins = select.select(list(self.socks.values()), [], [], 0.01)[0]
            except:
                print('Socket selection error!')

            for s in ins:
                if s in self.socks.values():
                    try:
                        if s == self.master_sock:
                            data = s.recv(4)
                            if len(data) != 4:
                                self.kill()

                            data = recv_all(s, struct.unpack('<I', data)[0])

                        else:
                            data = s.recv(65535)
                    except:
                        print('Recv error!')

                    if data == b'':
                        # lost master
                        if s == self.master_sock:
                            self.kill()
                            continue

                        # lost client
                        else:
                            bin_data = struct.pack('<BHH' + str(len(data)) + 's', CLOSE, s.sport-1, s.eport, b'\0')
                            bin_data = struct.pack('<I', len(bin_data)) + bin_dat
                            try:
                                self.master_sock.send(bin_data)
                            except:
                                print('Transmit error.')
                                self.kill()
                                continue

                        # remove port
                        del self.socks[s.eport]
                        s.close()
                        continue

                    else:
                        if s == self.master_sock:
                            data = struct.unpack('<BHH' + str(len(data[5:])) + 's', data)
                            data = {'cmd': data[0], 'sport': data[1], 'eport': data[2], 'payload': data[3]}

                            if data['cmd'] == OPEN:
                                #TODO: fix +1
                                self.connect('127.0.0.1', data['sport']+1, data['eport'])

                            elif data['cmd'] == CLOSE:
                                if data['eport'] in self.socks.keys():
                                    self.socks[data['eport']].close()
                                    del self.socks[data['eport']]

                            elif data['cmd'] == DATA:
                                if data['eport'] in self.socks.keys():
                                    try:
                                        self.socks[data['eport']].send(data['payload'])
                                    except:
                                        print('Transmit error.')
                                        self.socks[data['eport']].close()
                                        del self.socks[data['eport']]

                        else:
                            # TODO: fix -1
                            bin_data = struct.pack('<BHH' + str(len(data)) + 's', DATA, s.sport-1, s.eport, data)
                            bin_data = struct.pack('<I', len(bin_data)) + bin_data
                            try:
                                self.master_sock.send(bin_data)
                            except:
                                print('Transmit error')
                                self.kill()
                                continue
                else:
                    s.close()

        self.kill()

class Server(Thread):
    def __init__(self, port):
        super(Server, self).__init__()
        self.port = port
        self.master_listener = None
        self.listeners = {}
        self.add_port('0.0.0.0', 5555, True)
        self.running = True

    def add_port(self, ip, port, master=False):
        if port in self.listeners.keys():
            return False

        l = Listener(ip, port, master)

        if master:
            self.master_listener = l

        l.start()
        self.listeners[port] = l

    def run(self):
        while self.running:

            # poll pipes for data
            for l in self.listeners.values():
                while l.pipe[0].poll():
                    data = l.pipe[0].recv()

                    # start listeners
                    if data['cmd'] == READY:
                        for l2 in self.listeners.values():
                            if not l2.is_master:
                                l2.start_socket()

                    elif data['cmd'] == NOTREADY:
                        for l2 in self.listeners.values():
                            if not l2.is_master:
                                l2.kill()

                    elif data['cmd'] == OPEN or data['cmd'] == DATA or data['cmd'] == CLOSE:
                        if l == self.master_listener:
                            if data['sport'] in self.listeners.keys():
                                self.listeners[data['sport']].pipe[0].send(data)
                        else:
                            self.master_listener.pipe[0].send(data)

            time.sleep(0.05)

        print('Server shutting down.')
        for l in self.listeners.values():
            l.running = False
            l.join()

class Listener(Thread):
    def __init__(self, ip, port, master=False):
        super(Listener, self).__init__()
        self.ip = '0.0.0.0'
        self.pipe = Pipe(True)
        self.port = port
        self.is_master = master
        self.socks = {}
        self.running = True

    def kill(self):
        for s in self.socks.values():
            s.close()
        self.socks = {}

    def start_socket(self):
        print('Starting socket:', self.port)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setblocking(0)
            s.bind((self.ip, self.port))
            s.listen(25)
            self.socks[0] = s
        except:
            self.kill()
            return False

        return True

    def stop_socket(self):
        print('Stopping!', self.port)
        if 0 in self.socks.keys():
            self.socks[0].close()
            del self.socks[0]

    def close_socket(self, eport):
        if eport in self.socks.keys():
            self.socks[eport].close()
            del self.socks[eport] 

    def run(self):
        if self.is_master:
            self.start_socket()

        print('Started:', self.port)
        while self.running:
            # poll for data / connections
            ins = select.select(list(self.socks.values()), [], [], 0.01)[0]
            for s in ins:
                # new connection
                if 0 in self.socks.keys() and s == self.socks[0]:
                    c, addr = self.socks[0].accept()
                    print('Connection from: ', addr)

                    # only allow 1 master connection
                    if self.is_master:
                        self.stop_socket()
                        self.pipe[1].send({'sport': self.port, 'eport': addr[1], 'cmd': READY, 'payload': b'\0'})

                    # send connect request
                    else:
                        self.pipe[1].send({'sport': self.port, 'eport': addr[1], 'cmd': OPEN, 'payload': b'\0'})

                    self.socks[addr[1]] = c

                elif s in self.socks.values():
                    if self.is_master:
                        try:
                            data = s.recv(4)
                            if len(data) != 4:
                                data = b''

                            data = recv_all(s, struct.unpack('<I', data)[0])
                        except:
                            data = b''

                    else:
                        try:
                            data = s.recv(65535)
                        except:
                            data = b''

                    if data == b'':
                        # lost master
                        if self.is_master:
                            if not self.start_socket():
                                continue

                            self.pipe[1].send({'sport': s.getsockname()[1], 'eport': addr[1], 'cmd': NOTREADY, 'payload': b'\0'})

                        # lost client
                        else:
                            self.pipe[1].send({'sport': s.getsockname()[1], 'eport': s.getpeername()[1], 'cmd': CLOSE, 'payload': b'\0'})

                        # remove port
                        self.close_socket(s.getpeername()[1])
                        continue

                    else:
                        if self.is_master:
                            data = struct.unpack('<BHH' + str(len(data[5:])) + 's', data)
                            self.pipe[1].send({'cmd': data[0], 'sport': data[1], 'eport': data[2], 'payload': data[3]})
                        else:
                            self.pipe[1].send({'sport': s.getsockname()[1], 'eport': s.getpeername()[1], 'cmd': DATA, 'payload': data})

            # send out data to clients
            while self.pipe[1].poll():
                data = self.pipe[1].recv()

                if data['eport'] in self.socks.keys() or self.is_master:
                    if (self.is_master):
                        bin_data = struct.pack('<BHH' + str(len(data['payload'])) + 's', data['cmd'], data['sport'], data['eport'], data['payload'])
                        bin_data = struct.pack('<I', len(bin_data)) + bin_data
                        try:
                            self.socks[list(self.socks.keys())[0]].send(bin_data)
                        except:
                            self.close_socket(0)
                            self.pipe[1].send({'sport': s.getsockname()[1], 'eport': addr[1], 'cmd': NOTREADY, 'payload': b'\0'})
                            self.start_socket()
                    else:
                        if data['cmd'] == CLOSE:
                            self.close_socket(data['eport'])
                        elif data['cmd'] == DATA:
                            try:
                                self.socks[data['eport']].send(data['payload'])
                            except:
                                self.close_socket(data['eport'])

        # close all connections
        self.kill()

def main():
    global running
    signal.signal(signal.SIGINT, signal_handler)

    servers = []

    p = argparse.ArgumentParser()
    p.add_argument('mode', help='relay mode (client | server)')
    p.add_argument('-l', type=int, action='append', help='listener port')
    p.add_argument('-u', type=str, help='server ip')
    p.add_argument('-p', type=int, help='server port')
    args = p.parse_args()

    # mode
    if args.mode == 'client':
        if not args.u or not args.p:
            p.print_help(sys.stderr)
            sys.exit() 

        client_thread = Client(args.u, args.p)
        client_thread.start()

    elif args.mode == 'server':
        print('Starting server!')
        server_thread = Server(5555)
        server_thread.start()

        # ports
        if args.l:
            for l in args.l:
                print('Starting listener on: ', l)
                server_thread.add_port('0.0.0.0', l)
    else:
        p.print_help(sys.stderr)
        sys.exit()

    while running:
        time.sleep(0.1)

    print ('Shutting down!')
    if args.mode == 'server':
        server_thread.running = False
        server_thread.join()
    
if __name__ == "__main__":
    main()
