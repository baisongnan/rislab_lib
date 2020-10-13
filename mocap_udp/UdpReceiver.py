import socket
import time
import numpy as np
import threading
from tqdm import tqdm

from ..data_processor.GeneralFcn import MyRealTimeProcessor
from ..savemat import savemat


class UdpRigidBodies(object):
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222):
        self.udp_flag = 0
        self._udpStop = False
        self._udp_data = None
        self._udpThread = None
        self._udpThread_on = False
        self._udp_data_ready = threading.Event()
        self._synced_event = threading.Event()
        self._synced_event_2 = threading.Event()
        self._sync_on = False

        self.udp_ip = udp_ip
        self.udp_port = udp_port

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self._sock.bind((self.udp_ip, self.udp_port))

        self.sample_rate = -1  # flag
        self.sample_rate = self.get_sample_rate()
        self.sample_time = 1 / self.sample_rate
        print('UDP receiver initialized')

    def get_sample_rate(self):
        if self.sample_rate == -1:
            print('Computing sample rate...')
            time_list = []
            for i in tqdm(range(1000), desc="Processing...", leave=True, position=0):
                time_list.append(time.time())
                data, addr = self._sock.recvfrom(100)  # buffer size is 8192 bytes
            d_time = np.diff(time_list)
            sample_time = np.mean(d_time)

            print('Sample rate: ', '%.2f' % (1/sample_time), 'Hz')
            return 1/sample_time
        else:
            return self.sample_rate

    def start_thread(self):
        if not self._udpThread_on:
            self._udpThread = threading.Thread(target=self._udp_worker, args=(), )
            self._udp_data = 1
            self._udpThread.start()
            self._udpThread_on = True
            time.sleep(0.1)
            print('Upd thread start')
        else:
            print('New upd thread is not started')

    def _udp_worker(self, ):
        if not self._udpThread_on:
            print('Receive data without data processing')
            # main loop

            while not self._udpStop:
                self.udp_flag = self.udp_flag + 1
                self._synced_event.clear()
                udp_data_temp, addr = self._sock.recvfrom(100)  # buffer size is 8192 bytes
                self._udp_data_ready.clear()
                self._udp_data = udp_data_temp
                self._udp_data_ready.set()
                self._synced_event.set()
                if self._sync_on:
                    self._synced_event_2.wait()
                    self._synced_event_2.clear()
                # print(self.udp_flag)
            print('upd thread stopped')

    def stop_thread(self, ):
        self._sync_on = False
        self._udpStop = True

    def get_data(self, ):
        # get current data
        self._sync_on = False
        self._udp_data_ready.wait()
        return self._udp_data

    def sync_switch(self, sync_on):
        self._sync_on = sync_on

    def get_data_sync(self, ):
        # get data synced, this function may block your program by frequency of udp
        self._sync_on = True
        self._synced_event.wait()
        temp_data = self._udp_data
        self._synced_event_2.set()
        return temp_data


class MyCustomizedUdp(UdpRigidBodies):
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222, order=4, cutoff=16,
                 ftype='lowpass', design='cheby2', rs=58, filter_on=False):
        UdpRigidBodies.__init__(self, udp_ip=udp_ip, udp_port=udp_port, )
        self.data_processor = MyRealTimeProcessor(order, cutoff, ftype, design, rs, self.sample_rate, self.sample_time, filter_on)
        self.filter_on = False
        self.saver = savemat.DataSaver('Abs_time', 'X', 'Y', 'Z', 'QW', 'QX', 'QY', 'QZ')

    def start_thread(self):
        if not self._udpThread_on:
            self._udpThread = threading.Thread(target=self._udp_worker_sync, args=(), )
            self._udp_data = 1
            self._udpThread.start()
            self._udpThread_on = True
            time.sleep(0.1)
            print('Upd thread start')
        else:
            print('New upd thread is not started')

    def _udp_worker_sync(self, ):
        if not self._udpThread_on:
            print('Receive data with data processing')
            # main loop
            while not self._udpStop:
                self.udp_flag = self.udp_flag + 1
                udp_data_temp, addr = self._sock.recvfrom(100)  # buffer size is 8192 bytes
                self._udp_data_ready.clear()
                self._udp_data = udp_data_temp
                self.data_processor.step(self._udp_data, )
                self._udp_data_ready.set()
                self.saver.add_elements(time.time(),
                                        self.data_processor.X, self.data_processor.Y, self.data_processor.Z,
                                        self.data_processor.QW, self.data_processor.QX,
                                        self.data_processor.QY, self.data_processor.QZ, )
                # print(self.udp_flag)
            print('upd thread stopped')

    def get_data(self, ):
        # get current data
        self._sync_on = False
        self._udp_data_ready.wait()
        return self.data_processor

    def save_data(self, path):
        self.saver.save2mat(path)
