# -*- coding: utf-8 -*-
import os
import tkinter as tk
import cv2
from PIL import Image, ImageTk
import numpy as np
import shutil


class LovelyCalibTool:
    def __init__(self,
                 imgs=[[]],
                 board_size=(7, 7),
                 pattern=1,
                 physics_size=(16.4375, 15.875),
                 calib_save_to='.',
                 calib_save_name = 'calib.xml'):
        # board_size=(width,height)
        # board_size: inner corners, w - h, pattern 1: chess; 0: circle; physics_size:w-h
        self.camera_type = 'single' if len(imgs)==1 else 'stereo'
        self.imgs = imgs
        self.pattern = pattern
        self.board_size = board_size
        self.physics_size = physics_size
        self.aspect_ratio = 1.0
        self.win_size = (11, 11)
        self.grid_width = (physics_size[0] * (self.board_size[0] - 1), physics_size[1] * (self.board_size[1] - 1))
        self.img_size = (imgs[0][0].shape[1], imgs[0][0].shape[0])

        self.matrix = 0
        self.optimal_matrix = 0
        self.dist = 0
        self.rvecs = 0
        self.tvecs = 0
        self.save_to = calib_save_to
        self.calib_save_to = os.path.join(self.save_to, calib_save_name)

    def _cal_real_corner(self, corner_height, corner_width, physics_size):
        obj_corner = np.zeros([corner_height * corner_width, 3], np.float32)
        obj_corner[:, :2] = np.mgrid[0:corner_height, 0:corner_width].T.reshape(-1, 2)  # (w*h)*2
        obj_corner[:, 0:1] = obj_corner[:, 0:1] * physics_size[1]
        obj_corner[:, 1:2] = obj_corner[:, 1:2] * physics_size[0]
        return obj_corner

    def _calib_one_camera(self):
        imgs = self.imgs[0]
        imgs_corners = []
        objs_corners = []
        for img in imgs:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            assert (img_gray.shape[0] == self.img_size[1] and img_gray.shape[1] == self.img_size[0]), \
                "Image size does not match the given value {}.".format(self.img_size)
            ret, img_corners = cv2.findChessboardCorners(img_gray, (self.board_size[1], self.board_size[0]))
            if ret:
                obj_corners = self._cal_real_corner(self.board_size[1], self.board_size[0], self.physics_size)
                objs_corners.append(obj_corners)
                img_corners = cv2.cornerSubPix(img_gray,
                                               img_corners,
                                               winSize=self.win_size,
                                               zeroZone=(-1, -1),
                                               criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                imgs_corners.append(img_corners)
        ret, self.matrix, self.dist, rvecs, tveces = cv2.calibrateCamera(objs_corners, imgs_corners, self.img_size,
                                                                         None, None)
        if not (cv2.checkRange(self.matrix) and cv2.checkRange(self.dist)):
            print('ERROR IN CALIB')
        self.optimal_matrix, roi = cv2.getOptimalNewCameraMatrix(self.matrix, self.dist, self.img_size, alpha=1)
        # ret, self.matrix, self.dist, rvecs, tveces = cv2.calibrateCameraRO(objs_corners, imgs_corners, self.img_size, self.board_size[0]-1, self.matrix,self.dist)
        return True

    def _save_calib(self):
        save_calib = cv2.FileStorage(self.calib_save_to, cv2.FILE_STORAGE_WRITE)
        save_calib.write('camera_matrix', self.matrix)
        save_calib.write('optimal_matrix', self.optimal_matrix)
        save_calib.write('distortion', self.dist)
        save_calib.release()

    def _debug_test(self):
        imgs = self.imgs[0]
        count_n = 0
        for idx, img in enumerate(imgs):
            save_2 = os.path.join(self.save_to, 'undist_'+str(idx) + '.jpg')
            img_u = cv2.undistort(img, self.matrix, self.dist, self.optimal_matrix)
            cv2.imwrite(save_2, img_u)

    def Run(self):
        if self.camera_type == 'single':
            if self.pattern == 1:
                self._calib_one_camera()
                self._save_calib()
                self._debug_test()
        return