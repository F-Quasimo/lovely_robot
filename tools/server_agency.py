# -*- coding: utf-8 -*-
#!/usr/bin/python3
import socket
from threading import Thread
import time
from multiprocessing import Process
import subprocess
import numpy as np
import cv2
import sys

class RoboServer:
    def __init__(self, host='0.0.0.0', port=8888):
        self.socket_listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_listener.bind((host, port))
        self.socket_listener.listen(5)
        self.connected_pool = {}
        self.data_head_len = 1024

    def accept_thred(self):
        while True:
            client, address = self.socket_listener.accept()
            thread_handel = Thread(target=self.receiver_handle, args=(client, address))
            thread_handel.setDaemon(self)
            thread_handel.start()

    def _receive_all(self, skt, count):
        buf = b''
        while count:
            new_buf = skt.recv(count)
            if not new_buf:
                return None
            buf += new_buf
            count -= len(new_buf)
        return buf

    def _decode_head(self, head):
        data_len = int(head[0:16])
        data_idx = int(head[16:32])
        return {'data_len': data_len, 'data_idx': data_idx}

    def _deal_data_package(self, opt, client):
        print('_deal_data_package: start')
        data_recv = self._receive_all(skt=client, count=opt['data_len'])
        data_np = np.frombuffer(data_recv, np.uint8)
        decode_img = cv2.imdecode(data_np, cv2.IMREAD_COLOR)
        cv2.imshow('SERVER', decode_img)
        cv2.waitKey(10)
        #cv2.imwrite(str(opt['data_idx']).zfill(6) + '_need_delet.jpg', decode_img)

        # ################         PROCESS IN SERVER       #######################
        #

        client.sendall((str(opt['data_idx']) + ' deal~').ljust(1024).encode())
        print('_deal_data_package: end')

    def remove_client(self, address):
        client = self.connected_pool[address]
        if client is not None:
            client.close()
            self.connected_pool.pop(address)
            print('Client Close: ', address)

    def receiver_handle(self, client, address, **kwargs):
        address = str(address)
        print('client: ', address, ' connect')
        client.sendall('SERVER_OK'.ljust(1024).encode())
        self.connected_pool[address] = client

        while True:
            try:
                skt_head = self._receive_all(client, self.data_head_len)
                print('receiver_handle: skt_head', skt_head)
                if skt_head == '':
                    print('Client Close by send NULL')
                    self.remove_client(address)
                skt_head_op = self._decode_head(skt_head)
                print('receiver_handle: skt_head_op', skt_head_op)
                self._deal_data_package(opt=skt_head_op, client=client)

            except Exception as e:
                print('Exception: ', e)
                self.remove_client(address)
                break

    def Run(self):
        server_run = Thread(target=self.accept_thred)
        server_run.setDaemon(self)
        server_run.start()
        while True:
            time.sleep(0.1)


class RoboClient:
    def __init__(self, host, port=8888):
        self.skt_sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data_head_len = 1024
        self.data_return = []
        try:
            self.skt_sender.connect((host, port))
            ret = self.skt_sender.recv(1024).decode()
            print('RoBoClient ', ret)
        except socket.error as msg:
            print('Client connect error ', msg)
            sys.exit(1)

    def _thread_sender(self, data_head, data_in):
        print('Client thread_sender data_head', data_head)
        data_head_str = data_head['data_len'].ljust(16) + data_head['data_idx'].ljust(16)
        self.skt_sender.send(str.encode(data_head_str).ljust(self.data_head_len))
        self.skt_sender.send(data_in)
        receive = self.skt_sender.recv(1024)
        self.data_return.append([data_head['data_idx'], receive.decode()])

    def Send(self, frame, frame_idx=0):
        result, imgencode = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 15])
        encode_data = np.array(imgencode)
        str_data = encode_data.tostring()
        data_head = {'data_len': str(len(str_data)), 'data_idx': str(frame_idx)}
        thread_sender = Thread(target=self._thread_sender, args=(data_head, str_data))
        thread_sender.start()

    def Recv(self, frame_idx):
        sleep_count = 20
        while len(self.data_return) < frame_idx + 1 and sleep_count > 0:
            time.sleep(1)
            sleep_count = sleep_count - 1
        if sleep_count < 1 or len(self.data_return) < frame_idx + 1:
            return ''
        if self.data_return[frame_idx] is not None:
            return self.data_return[frame_idx]
        return ''


if __name__ == '__main__':
    server_main = RoboServer(host='0.0.0.0', port=8888)
    server_main.Run()