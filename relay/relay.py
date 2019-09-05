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
NEW = 6

running = True

class CustomSocket(socket.socket):
    def __init__(self, *args, **kwargs):
        super(CustomSocket, self).__init__(*args, **kwargs)
        self.eport = 0
        self.sport = 0

    @classmethod
    def copy(self, sock):
        print(sock.fileno())
        fd = socket.dup(sock.fileno())
        copy = self(sock.family, sock.type, sock.proto, fileno=fd)
        copy.settimeout(sock.gettimeout())
        return copy


def signal_handler(signal, frame):
    global running
    running = False

class SocketThread(Thread):
    def __init__(self, ip=None, port=None, eport=0, is_master=False, is_listener=False, s=None):
        super(SocketThread, self).__init__()
        self.s = s
        self.ip = ip
        self.eport = eport
        self.sport = port
        self.pipe = Pipe(True)
        self.is_master = is_master
        self.is_listener = is_listener
        self.running = True
        self.is_connected = False
        if s:
            self.is_connected = True

    def connect(self):
        try:
            print ('Opened:', self.eport, self.sport)
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.ip, self.sport))

            self.is_connected = True
            return True

        except:
            print('Error connecting to server.')
            return False

    def listen(self):
        print('Starting socket:', self.ip, self.sport)
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.s.bind((self.ip, self.sport))
            if self.is_master:
                self.s.listen(1)
            else:
                self.s.listen(25)
        except:
            print('Error listening on port:', self.sport)
            return False

        return True

    def kill(self):
        if self.s:
            self.s.close()
        self.is_connected = False
        self.running = False
        return

    def send_all(self, data, timeout=10):
        start_time = time.time()

        while len(data) > 0 and self.running:
            outs = select.select([], [self.s], [], 0.01)[1]
            if self.s in outs:
                sent_len = self.s.send(data)
                data = data[sent_len:]
                
                start_time = time.time()

            elif time.time() - start_time > timeout:
                return False

        return True

    def recv(self, data_len):
        data = b''
        try:
            data = self.s.recv(data_len)
        except:
            return b''
        return data

    def recv_all(self, data_len, timeout=10):
        data = b''
        start_time = time.time()

        print('Getting:', data_len)
        while len(data) < data_len and self.running:
            ins = select.select([self.s], [], [], 0.01)[0]
            if self.s in ins:
                t_data = self.recv(data_len - len(data))

                if t_data == b'':
                    return b''

                data += t_data

                start_time = time.time()

            elif time.time() - start_time > timeout:
                return b''

        if len(data) < data_len:
            return b''

        return data

    def run(self):
        if self.is_listener:
            self.listen()
            while self.running:
                ins = select.select([self.s, self.pipe[1]], [], [], 0.5)[0]
                if self.s in ins:
                    try:
                        c, addr = self.s.accept()
                    except:
                        print('Socket accept error!')
                        self.running = False
                        break

                    print('Connection from: ', addr)
                    self.pipe[1].send({'s': c, 'sport': self.sport, 'eport': addr[1], 'cmd': NEW, 'master': self.is_master, 'payload': b'\0'})
                    print('sent!')

        else:
            print(self.running, self.is_master, self.is_connected)
            while self.running:
                if not self.is_connected and not self.connect():
                    if self.is_master:
                        time.sleep(1)
                        continue
                    else:
                        break

                else:
                    print('here2')
                    ins = select.select([self.s, self.pipe[1]], [], [], 0.5)[0]
                    if self.s in ins:
                        if self.is_master:
                            data = self.recv_all(4)
                            if len(data) == 4:
                                data = self.recv_all(struct.unpack('<I', data)[0])

                        else:
                            data = self.recv(65535)

                        if data == b'':
                            # lost master
                            if self.is_master:
                                self.s.close()
                                self.is_connected = False
                                continue

                            # lost client
                            else:
                                bin_data = struct.pack('<BHH' + str(len(data)) + 's', CLOSE, self.sport, self.eport, b'\0')
                                bin_data = struct.pack('<I', len(bin_data)) + bin_data
                                self.pipe[1].send(bin_data)
                                self.kill()

                        else:
                            out_data = None
                            if self.is_master:
                                data = struct.unpack('<BHH' + str(len(data[5:])) + 's', data)
                                out_data = {'cmd': data[0], 'sport': data[1], 'eport': data[2], 'payload': data[3]}

                            else:
                                out_data = struct.pack('<BHH' + str(len(data)) + 's', DATA, self.sport, self.eport, data)
                                out_data = struct.pack('<I', len(out_data)) + out_data

                            self.pipe[1].send(out_data)

                    if self.pipe[1] in ins:
                        data = self.pipe[1].recv()
                        self.send_all(data)

        self.kill()


class Relay(Thread):
    def __init__(self, ip, port, is_server=False):
        super(Relay, self).__init__()
        self.server_port = port
        self.server_ip = ip
        self.master_socket = SocketThread(ip, port, is_master=True, is_listener=is_server)
        self.master_pipe = self.master_socket.pipe[0]
        self.socks = {}
        self.pipes = {}
        self.running = True

    def kill(self):
        for s in self.socks.values():
            s.running = False
            s.kill()
            s.join()
        self.master_socket.running = False
        self.master_socket.join()

    def add_listener(self, ip, port):
        if port not in self.socks.keys():
            st = SocketThread(ip, port, is_master=False, is_listener=True)
            st.start()
            self.socks[port] = st
            self.pipes[port] = st.pipe[0]
            return True

        return False

    def run(self):
        self.master_socket.start()
        while self.running:
            print('here', self.pipes)
            inp = select.select([ self.master_pipe ] + list(self.pipes.values()), [], [], 0.5)[0]
            for p in inp:
                print(p)
                #data from master
                if p == self.master_socket.pipe[0]:
                    data = p.recv()
                    print(data)
                    if data['cmd'] == OPEN:
                        if data['eport'] not in self.socks.keys():
                            st = SocketThread('127.0.0.1', data['sport'], data['eport'])
                            st.start()
                            self.socks[data['eport']] = st
                            self.pipes[data['eport']] = st.pipe[0]

                    elif data['cmd'] == CLOSE:
                        if data['eport'] in self.socks.keys():
                            self.socks[data['eport']].kill()
                            del self.pipes[data['eport']]
                            self.socks[data['eport']].running = False
                            self.socks[data['eport']].join()
                            del self.socks[data['eport']]

                    elif data['cmd'] == DATA:
                        if data['eport'] in self.socks.keys() and data['eport'] in self.pipes.keys():
                            try:
                                self.pipes[data['eport']].send(data['payload'])
                            except:
                                self.socks[data['eport']].kill()
                                del self.pipes[data['eport']]
                                self.socks[data['eport']].running = False
                                self.socks[data['eport']].join()
                                del self.socks[data['eport']]

                    elif data['cmd'] == NEW:
                        print('new')
                        st = SocketThread(port=data['sport'], eport=data['eport'], s=data['s'])

                        # only one master, yo
                        if data['master'] and 0 in self.socks.keys():
                            print('nother master')
                            st.running = False
                            st.kill()
                            continue

                        st.is_master = data['master']
                        st.start()
                        self.socks[0] = st
                        self.pipes[0] = st.pipe[0]
                        print(self.socks)

                #data from clients
                else:
                    data = p.recv()
                    self.master_socket.pipe[0].send(data)

        self.kill()

'''
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
                    co, addr = self.socks[0].accept()
                    c = CustomSocket.copy(co)
                    c.eport = addr[1]
                    c.sport = s.getsockname()[1]
                    co.close()
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
                            data = recv_all(s, 4)
                            if len(data) == 4:
                                data = recv_all(s, struct.unpack('<I', data)[0])
                        except:
                            data = b''

                    else:
                        try:
                            data = recv(s, 65535)
                        except:
                            data = b''

                    if data == b'':
                        # lost master
                        if self.is_master:
                            if not self.start_socket():
                                continue

                            self.pipe[1].send({'sport': s.sport, 'eport': s.eport, 'cmd': NOTREADY, 'payload': b'\0'})

                        # lost client
                        else:
                            self.pipe[1].send({'sport': s.sport, 'eport': s.eport, 'cmd': CLOSE, 'payload': b'\0'})

                        # remove port
                        self.close_socket(s.eport)
                        continue

                    else:
                        if self.is_master:
                            data = struct.unpack('<BHH' + str(len(data[5:])) + 's', data)
                            self.pipe[1].send({'cmd': data[0], 'sport': data[1], 'eport': data[2], 'payload': data[3]})
                        else:
                            self.pipe[1].send({'sport': s.sport, 'eport': s.eport, 'cmd': DATA, 'payload': data})

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
                            self.pipe[1].send({'sport': s.sport, 'eport': s.eport, 'cmd': NOTREADY, 'payload': b'\0'})
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
'''

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

        client_thread = Relay(args.u, args.p)
        client_thread.start()

    elif args.mode == 'server':
        print('Starting server!')
        server_thread = Relay('0.0.0.0', 5555, is_server=True)
        server_thread.start()

        # ports
        if args.l:
            for l in args.l:
                print('Starting listener on: ', l)
                server_thread.add_listener('0.0.0.0', l)
    else:
        p.print_help(sys.stderr)
        sys.exit()

    while running:
        time.sleep(0.1)

    print ('Shutting down!')
    if args.mode == 'server':
        server_thread.running = False
        server_thread.join()

    else:
        client_thread.running = False
        client_thread.join()
    
if __name__ == "__main__":
    main()
