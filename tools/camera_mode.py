# -*- coding: utf-8 -*-
import os
import cv2
import numpy as np
import platform


class SingleCam:
    def __init__(self, cam_id=0, cam_size=(1920, 1080), cam_mode=cv2.CAP_DSHOW):
        self.cam_id = cam_id
        self.cap = None
        self.cam_size = cam_size
        self.cap_open_mode = cam_mode

    def OpenCam(self):
        self.cap = cv2.VideoCapture(self.cam_id + self.cap_open_mode)
        self.cap.open(self.cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_size[1])
        print('DEBUG SingleCam OpenCam')
        if self.cap.isOpened():
            return True
        else:
            self.cap = None
            print('DEBUG SingleCam OpenCam FAIL')
            return False

    def SnapShoot(self):
        if self.cap != None and self.cap.isOpened():
            ret, snap = self.cap.read()
            return snap
        else:
            return None

    def Close(self):
        if self.cap.isOpened():
            self.cap.release()
            self.cap = None


class StereoCam:
    def __init__(self, cam_id0, cam_id1, cam_size=(1920, 1080), cam_mode=cv2.CAP_DSHOW):
        self.cam_id0 = cam_id0
        self.cam_id1 = cam_id1
        self.cap0 = None
        self.cap1 = None
        self.cam_size = cam_size
        self.cap_open_mode = cam_mode

    def OpenCam(self):
        self.cap0 = cv2.VideoCapture(self.cam_id0 + self.cap_open_mode)
        self.cap0.open(self.cam_id0)
        self.cap0.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_size[0])
        self.cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_size[1])
        self.cap1 = cv2.VideoCapture(self.cam_id1 + self.cap_open_mode)
        self.cap1.open(self.cam_id1)
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_size[0])
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_size[1])
        if self.cap0.isOpened() and self.cap1.isOpened():
            return True
        else:
            self.Close()
            return False

    def SnapShoot(self):
        if self.cap0 != None and self.cap0.isOpened() and self.cap1 != None and self.cap1.isOpened():
            ret, snap0 = self.cap0.read()
            ret, snap1 = self.cap1.read()
            return snap0, snap1
        else:
            return None, None

    def Close(self):
        if self.cap0 is not None:
            if self.cap0.isOpened():
                self.cap0.release()
                self.cap0 = None
        if self.cap1 is not None:
            if self.cap1.isOpened():
                self.cap1.release()
                self.cap1 = None