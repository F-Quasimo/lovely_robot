# -*- coding: utf-8 -*-
import threading
import serial
import ctypes
import inspect
import time


class RoboSerial:

    def __init__(
        self,
        com_port='COM0',
        baud_rate=115200,
        parity_check=serial.PARITY_NONE,
        stop_bits=1,
        byte_size=serial.EIGHTBITS,
        timeout=0.2,
        receiver_callbacks=[],
        unibus=True
    ):
        self.port = com_port
        self.baud_rate = baud_rate
        self.parity_check = parity_check
        self.stop_bits = stop_bits
        self.byte_size = byte_size
        self.timeout = timeout
        self.receiver_thread_flag = False
        self.receiver_thread = None
        self.receiver_buffer = ''
        self.my_serial = None
        self.receiver_callbacks = receiver_callbacks
        self.unibus = unibus
        self.sending_flag = False

    def SetReceiverCallBack(self, call_backs):
        self.receiver_callbacks = call_backs
        return

    def SetReceiverDefaultCallBack(self):
        def _default_callback(**kwargs):
            return
        self.receiver_callbacks = []

    def IsOpen(self):
        if self.my_serial == None:
            return False
        else:
            return self.my_serial.isOpen()

    def _thread_receive(self):
        while self.receiver_thread_flag:
            if self.unibus and self.sending_flag:
                continue
            else:
                read = self.my_serial.readall()
                if len(read) > 0:
                    self.receiver_buffer = str(bytes(read).decode('utf-8', "ignore"))
                    for func in self.receiver_callbacks:
                        func(self.receiver_buffer)
                    print(self.receiver_buffer, ' ', self.receiver_callbacks)

    def Open(self):
        self.my_serial = serial.Serial(port=self.port,
                                       baudrate=self.baud_rate,
                                       parity=self.parity_check,
                                       timeout=self.timeout,
                                       stopbits=self.stop_bits,
                                       bytesize=self.byte_size)
        self.receiver_thread_flag = True
        self.receiver_thread = threading.Thread(target=self._thread_receive)
        self.receiver_thread.start()
        # self.receiver_callback('Open OK')

    def Send(self, buffer):
        if self.unibus:
            self.sending_flag = True
        status_ = self.my_serial.isOpen()
        if status_ == True:
            print('DEBUG BUFFER SEND: ', buffer.encode('ascii'))
            self.my_serial.write(buffer.encode('ascii'))
            self.my_serial.flushInput()
        else:
            print('ERROR IN SERIAL SEND! SERIAL IS NOT OPEN\n')
        if self.unibus:
            self.receiver_buffer = ''
            self.sending_flag = False

    def _async_raise(self, tid, exctype):
        """raises the exception, performs cleanup if needed"""
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            # """if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"""
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    def Close(self):
        self.receiver_thread_flag = False
        self.my_serial.close()
        # self._async_raise(self.receiver_thread.ident, SystemExit)
        
if __name__ == '__main__':
    robo_serial = RoboSerial(com_port='/dev/ttyS0',unibus=False)
    robo_serial.Open()
    robo_serial.Send(buffer='SEND And Open Serial OK')
    while True:
        time.sleep(2)
