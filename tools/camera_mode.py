# -*- coding: utf-8 -*-
import os
import cv2
import numpy as np
import platform

class SingleCam:

    def __init__(self, cam_id=0) :
        self.cam_id = cam_id
        self.cap = None


    def OpenCam(self):
        self.cap = cv2.VideoCapture(self.cam_id)
        if self.cap.isOpened():
            return True
        else:
            self.cap = None
            return False


    def SnapShoot(self):
        if self.cap != None and self.cap.isOpened():
            ret, snap = self.cap.read()
            return snap

    def Close(self):
        if self.cap.isOpened():
            self.cap.release()
            self.cap = None

class StereoCam:

    def __init_(self, cam_id0, cam_id1):
        self.cam_id0 = cam_id0
        self.cam_id1 = cam_id1
        self.cap0 = None
        self.cap1 = None

    def OpenCam(self):
        self.cap0 = cv2.VideoCapture(self.cam_id0)
        self.cap1 = cv2.VideoCapture(self.cam_id1)
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

    def Close(self):
        if self.cap0.isOpened():
            self.cap0.release()
            self.cap0 = None
            self.cap1 = None
        if self.cap1.isOpened():
            self.cap1.release()
            self.cap1 = None