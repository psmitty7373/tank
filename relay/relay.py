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
        self.running = True
        self.ready = False
        self.is_master = is_master
        self.is_listener = is_listener
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
                    if self.is_master:
                        if not self.ready:
                            self.ready = True
                            self.pipe[1].send({'s': c, 'sport': self.sport, 'eport': addr[1], 'cmd': NEW, 'master': True, 'payload': b'\0'})
                            self.pipe[1].send({'sport': self.sport, 'eport': addr[1], 'cmd': READY, 'payload': b'\0'})
                        else:
                            print('kill')
                            c.close()

                    else:
                        if self.ready:
                            self.pipe[1].send({'s': c, 'sport': self.sport, 'eport': addr[1], 'cmd': NEW, 'master': False, 'payload': b'\0'})
                        else:
                            c.close()

        else:
            while self.running:
                if not self.is_connected and not self.connect():
                    if self.is_master:
                        time.sleep(1)
                        continue
                    else:
                        self.pipe[1].send({'sport': s.sport, 'eport': s.eport, 'cmd': READY, 'payload': b'\0'})
                        self.ready = True
                        break

                else:
                    ins = select.select([self.s, self.pipe[1]], [], [], 0.01)[0]
                    if self.s in ins:
                        if self.is_master:
                            data = self.recv_all(4)
                            if len(data) == 4:
                                data = self.recv_all(struct.unpack('<I', data)[0])

                        else:
                            data = self.recv(65535)

                        # connection lost
                        if data == b'':
                            # lost master
                            if self.is_master:
                                self.ready = False
                                self.pipe[1].send({'sport': self.sport, 'eport': self.eport, 'cmd': NOTREADY, 'payload': b'\0'})
                                if self.ip:
                                    self.s.close()
                                    self.is_connected = False
                                    continue
                                else:
                                    self.kill()

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

                        if type(data) == dict:
                            data = struct.pack('<BHH' + str(len(data['payload'])) + 's', data['cmd'], data['sport'], data['eport'], data['payload'])
                            data = struct.pack('<I', len(data)) + data
     
                        self.send_all(data)

        self.kill()


class Relay(Thread):
    def __init__(self, ip, port, is_server=False):
        super(Relay, self).__init__()
        self.server_port = port
        self.server_ip = ip
        self.master_socket = SocketThread(ip, port, is_master=True, is_listener=is_server)
        if is_server:
            self.pipes = { port: self.master_socket.pipe[0] }
        else:
            self.pipes = { 'master':  self.master_socket.pipe[0] }
        self.socks = {}
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
            inp = select.select(list(self.pipes.values()), [], [], 0.5)[0]
            for p in inp:
                data = p.recv()
                #data from master
                if type(data) == dict:
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

                    elif data['cmd'] == NOTREADY:
                        print('not ready')
                        for st in self.socks.values():
                            if st.is_listener:
                                st.ready = False
                            else:
                                st.running = False
                                st.kill()
                                st.join()

                    elif data['cmd'] == READY:
                        print('ready')
                        for st in self.socks.values():
                            if st.is_listener:
                                st.ready = True

                    elif data['cmd'] == NEW:
                        st = SocketThread(port=data['sport'], eport=data['eport'], s=data['s'])

                        if data['master']:
                            st.is_master = True
                            st.start()
                            self.socks['master'] = st
                            self.pipes['master'] = st.pipe[0]
                                
                        else:
                            if 'master' in self.pipes.keys():
                                self.pipes['master'].send({'sport': data['sport'], 'eport': data['eport'], 'cmd': OPEN, 'payload': b'\0'})                            
                                st.start()
                                self.socks[data['eport']] = st
                                self.pipes[data['eport']] = st.pipe[0]

                            else:
                                st.running = False
                                st.kill()


                #data from clients
                else:
                    if 'master' in self.pipes.keys():
                        self.pipes['master'].send(data)

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
