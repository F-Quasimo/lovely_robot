# -*- coding: utf-8 -*-

import enum
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
                 calib_save_name='calib.xml'):
        # board_size=(width,height)
        # board_size: inner corners, w - h, pattern 1: chess; 0: circle; physics_size:w-h
        self.camera_type = 'single' if len(imgs) == 1 else 'stereo'
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

    def CalibOneCamera(self):
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
        return self.matrix, self.dist, self.optimal_matrix

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
            save_2 = os.path.join(self.save_to, 'undist_' + str(idx) + '.jpg')
            img_u = cv2.undistort(img, self.matrix, self.dist, self.optimal_matrix)
            cv2.imwrite(save_2, img_u)

    def Run(self):
        if self.camera_type == 'single':
            if self.pattern == 1:
                self.CalibOneCamera()
                self._save_calib()
                self._debug_test()
        return


class CalibStereo:
    # impl stereo calib
    def __init__(self,
                 imgs=[[], []],
                 board_size=(7, 7),
                 pattern=1,
                 physics_size=(16.4375, 15.875),
                 calib_save_to='.',
                 calib_save_name='calib_stereo.xml'):
        self.camera_type = 'stereo' if len(imgs) == 1 else 'single'
        self.imgs = imgs
        self.pattern = pattern
        self.board_size = board_size
        self.physics_size = physics_size
        self.aspect_ratio = 1.0
        self.win_size = (11, 11)
        self.grid_width = (physics_size[0] * (self.board_size[0] - 1), physics_size[1] * (self.board_size[1] - 1))
        self.img_size = (imgs[0][0].shape[1], imgs[0][0].shape[0])
        self.save_to = calib_save_to
        self.calib_save_to = os.path.join(self.save_to, calib_save_name)
        self.calib_tmp_files_save_to = os.path.join(self.save_to, 'tmp')
        if os.path.exists(self.calib_tmp_files_save_to):
            shutil.rmtree(self.calib_tmp_files_save_to)
        os.makedirs(self.calib_tmp_files_save_to)

        self.cameraMatrix_0 = None
        self.distCoeffs_0 = None
        self.optimal_matrix_0 = None
        self.cameraMatrix_1 = None
        self.distCoeffs_1 = None
        self.optimal_matrix_1 = None

        self.cameraMatrix0 = None
        self.distCoeffs0 = None
        self.cameraMatrix1 = None
        self.distCoeffs1 = None
        self.R_mat = None
        self.T_mat = None
        self.E_mat = None
        self.F_mat = None
        self.cam0_rectify = None
        self.cam1_rectify = None
        self.cam0_proj = None
        self.cam1_proj = None
        self.cam_Q_mat = None
        self.validPixROI0 = None
        self.validPixROI1 = None

        self.cam0_mapx = None
        self.cam0_mapy = None
        self.cam1_mapx = None
        self.cam1_mapy = None

        self.img_points0 = []
        self.img_points1 = []
        self.obj_points = []

    def _cal_real_corner(self, corner_height, corner_width, physics_size):
        obj_corner = np.zeros([corner_height * corner_width, 3], np.float32)
        obj_corner[:, :2] = np.mgrid[0:corner_height, 0:corner_width].T.reshape(-1, 2)  # (w*h)*2
        obj_corner[:, 0:1] = obj_corner[:, 0:1] * physics_size[1]
        obj_corner[:, 1:2] = obj_corner[:, 1:2] * physics_size[0]
        return obj_corner

    def Run(self):
        self.calib_two_cam()
        self.SaveCalib()

    def CalibOneCamera(self, cam_id):
        imgs = self.imgs[cam_id]
        imgs_corners = []
        objs_corners = []
        for idx, img in enumerate(imgs):
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
                if len(img_corners) != self.board_size[0] * self.board_size[1]:
                    print('ERROR CORNER NUM: ', idx)
                imgs_corners.append(img_corners)
        ret, cameraMatrix, distCoeffs, rvecs, tveces = cv2.calibrateCamera(objs_corners, imgs_corners, self.img_size,
                                                                           None, None)
        if not (cv2.checkRange(cameraMatrix) and cv2.checkRange(distCoeffs)):
            print('ERROR IN CALIB')
        optimal_matrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, self.img_size, alpha=1)
        # ret, self.matrix, self.dist, rvecs, tveces = cv2.calibrateCameraRO(objs_corners, imgs_corners, self.img_size, self.board_size[0]-1, self.matrix,self.dist)
        return cameraMatrix, distCoeffs, optimal_matrix

    def calib_two_cam(self):
        # cameraMatrix0 = cv2.initCameraMatrix2D(objectPoints=objs_corners,
        #                                        imagePoints=imgs0_corners,
        #                                        imageSize=self.img_size,
        #                                        aspectRatio=0)
        # cameraMatrix1 = cv2.initCameraMatrix2D(objectPoints=objs_corners,
        #                                        imagePoints=imgs1_corners,
        #                                        imageSize=self.img_size,
        #                                        aspectRatio=0)
        # ret, cameraMatrix0, distCoeffs0, rvecs, tveces = cv2.calibrateCamera(objs_corners, imgs0_corners, self.img_size,
        #                                                                      None, None)
        # ret, cameraMatrix1, distCoeffs1, rvecs, tveces = cv2.calibrateCamera(objs_corners, imgs1_corners, self.img_size,
        #                                                                      None, None)
        # optimal_matrix0, roi0 = cv2.getOptimalNewCameraMatrix(cameraMatrix0, distCoeffs0, self.img_size, alpha=1)
        # optimal_matrix1, roi1 = cv2.getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, self.img_size, alpha=1)
        cameraMatrix_0, distCoeffs_0, optimal_matrix_0 = self.CalibOneCamera(0)
        cameraMatrix_1, distCoeffs_1, optimal_matrix_1 = self.CalibOneCamera(1)
        objs_corners = []
        imgs0_corners = []
        imgs1_corners = []
        img_size = self.img_size
        for idx, (img0, img1) in enumerate(zip(self.imgs[0], self.imgs[1])):
            img_gray0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
            img_gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            assert (img_gray0.shape[0] == self.img_size[1] and img_gray0.shape[1] == self.img_size[0]), \
                "Image size does not match the given value {}.".format(self.img_size)
            ret0, img0_corners = cv2.findChessboardCorners(img_gray0, (self.board_size[1], self.board_size[0]))
            ret1, img1_corners = cv2.findChessboardCorners(img_gray1, (self.board_size[1], self.board_size[0]))
            if ret1 and ret0:
                obj_corners = self._cal_real_corner(self.board_size[1], self.board_size[0], self.physics_size)
                objs_corners.append(obj_corners)
                img0_corners = cv2.cornerSubPix(img_gray0,
                                                img0_corners,
                                                winSize=self.win_size,
                                                zeroZone=(-1, -1),
                                                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30,
                                                          0.001))
                imgs0_corners.append(img0_corners)
                img1_corners = cv2.cornerSubPix(img_gray0,
                                                img1_corners,
                                                winSize=self.win_size,
                                                zeroZone=(-1, -1),
                                                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30,
                                                          0.001))
                imgs1_corners.append(img1_corners)
            # draw findCorners, so you can easily check the corner is wannted or not
            img0_color = cv2.cvtColor(img_gray0, cv2.COLOR_GRAY2BGR)
            img1_color = cv2.cvtColor(img_gray1, cv2.COLOR_GRAY2BGR)
            draw_corners_0 = cv2.drawChessboardCorners(image=img0_color,
                                                       patternSize=self.board_size,
                                                       corners=img0_corners,
                                                       patternWasFound=ret0)
            draw_corners_1 = cv2.drawChessboardCorners(image=img1_color,
                                                       patternSize=self.board_size,
                                                       corners=img1_corners,
                                                       patternWasFound=ret1)
            cv2.imwrite(os.path.join(self.calib_tmp_files_save_to, 'draw_corner_' + str(idx).zfill(4) + '_img0.jpg'),
                        draw_corners_0)
            cv2.imwrite(os.path.join(self.calib_tmp_files_save_to, 'draw_corner_' + str(idx).zfill(4) + '_img1.jpg'),
                        draw_corners_1)

        retval, cameraMatrix0, distCoeffs0, cameraMatrix1, distCoeffs1, R_matrix, T_matrix, E_matrix, F_matrix = cv2.stereoCalibrate(
            objectPoints=objs_corners,
            imagePoints1=imgs0_corners,
            imagePoints2=imgs1_corners,
            cameraMatrix1=cameraMatrix_0,
            cameraMatrix2=cameraMatrix_1,
            distCoeffs1=distCoeffs_0,
            distCoeffs2=distCoeffs_1,
            imageSize=img_size,
            flags=cv2.CALIB_FIX_INTRINSIC,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5))

        R0_matrix, R1_matrix, P0_matrix, P1_matrix, Q_matrix, validPixROI0, validPixROI1 = cv2.stereoRectify(
            cameraMatrix1=cameraMatrix0,
            distCoeffs1=distCoeffs0,
            cameraMatrix2=cameraMatrix1,
            distCoeffs2=distCoeffs1,
            imageSize=img_size,
            R=R_matrix,
            T=T_matrix,
            alpha=0,
            flags=0)
        map0_x, map0_y = cv2.initUndistortRectifyMap(cameraMatrix=cameraMatrix0,
                                                     distCoeffs=distCoeffs0,
                                                     R=R0_matrix,
                                                     newCameraMatrix=P0_matrix,
                                                     size=img_size,
                                                     m1type=cv2.CV_32FC1)
        map1_x, map1_y = cv2.initUndistortRectifyMap(cameraMatrix=cameraMatrix1,
                                                     distCoeffs=distCoeffs1,
                                                     R=R1_matrix,
                                                     newCameraMatrix=P1_matrix,
                                                     size=img_size,
                                                     m1type=cv2.CV_32FC1)

        for idx, (img0, img1) in enumerate(zip(self.imgs[0], self.imgs[1])):
            new_img0 = cv2.remap(img0, map1=map0_x, map2=map0_y, interpolation=cv2.INTER_AREA)
            new_img1 = cv2.remap(img1, map1=map1_x, map2=map1_y, interpolation=cv2.INTER_AREA)

            combine = np.hstack([new_img0, new_img1])
            for line_idx in range(20):
                combine = cv2.line(combine, (0, int(img_size[1] / 20 * line_idx)),
                                   (int(img_size[0] * 2), int(img_size[1] / 20 * line_idx)), (0, 0, 255), 2, cv2.LINE_4)
            cv2.imwrite(os.path.join(self.calib_tmp_files_save_to, 'combine_' + str(idx).zfill(4) + '.jpg'), combine)
            img_u = cv2.undistort(img0, cameraMatrix=cameraMatrix0, distCoeffs=distCoeffs0)
            cv2.imwrite(os.path.join(self.calib_tmp_files_save_to, 'undist_main_' + str(idx).zfill(4) + '.jpg'), img_u)
            img_u = cv2.undistort(img1, cameraMatrix=cameraMatrix1, distCoeffs=distCoeffs1)
            cv2.imwrite(os.path.join(self.save_to, 'tmp', 'undist_aux_' + str(idx).zfill(4) + '.jpg'), img_u)
        # save to member
        self.cameraMatrix_0 = cameraMatrix_0
        self.distCoeffs_0 = distCoeffs_0
        self.optimal_matrix_0 = optimal_matrix_0
        self.cameraMatrix_1 = cameraMatrix_1
        self.distCoeffs_1 = distCoeffs_1
        self.optimal_matrix_1 = optimal_matrix_1

        self.cameraMatrix0 = cameraMatrix0
        self.distCoeffs0 = distCoeffs0
        self.cameraMatrix1 = cameraMatrix1
        self.distCoeffs1 = distCoeffs1
        self.R_mat = R_matrix
        self.T_mat = T_matrix
        self.E_mat = E_matrix
        self.F_mat = F_matrix
        self.cam0_rectify = R0_matrix
        self.cam1_rectify = R1_matrix
        self.cam0_proj = P0_matrix
        self.cam1_proj = P1_matrix
        self.cam_Q_mat = Q_matrix
        self.validPixROI0 = validPixROI0
        self.validPixROI1 = validPixROI1

        self.cam0_mapx = map0_x
        self.cam0_mapy = map0_y
        self.cam1_mapx = map1_x
        self.cam1_mapy = map1_y
        return

    def SaveCalib(self):
        save_calib = cv2.FileStorage(self.calib_save_to, cv2.FILE_STORAGE_WRITE)
        save_calib.write('cameraMatrix_0', self.cameraMatrix_0)
        save_calib.write('distCoeffs_0', self.distCoeffs_0)
        save_calib.write('optimal_matrix_0', self.optimal_matrix_0)
        save_calib.write('cameraMatrix_1', self.cameraMatrix_1)
        save_calib.write('distCoeffs_1', self.distCoeffs_1)
        save_calib.write('optimal_matrix_1', self.optimal_matrix_1)

        save_calib.write('cameraMatrix0', self.cameraMatrix0)
        save_calib.write('distCoeffs0', self.distCoeffs0)
        save_calib.write('cameraMatrix1', self.cameraMatrix1)
        save_calib.write('distCoeffs1', self.distCoeffs1)
        save_calib.write('R_mat', self.R_mat)
        save_calib.write('T_mat', self.T_mat)
        save_calib.write('E_mat', self.E_mat)
        save_calib.write('F_mat', self.F_mat)
        save_calib.write('cam0_rectify', self.cam0_rectify)
        save_calib.write('cam1_rectify', self.cam1_rectify)
        save_calib.write('cam0_proj', self.cam0_proj)
        save_calib.write('cam1_proj', self.cam1_proj)
        save_calib.write('cam_Q_mat', self.cam_Q_mat)
        save_calib.write('validPixROI0', self.validPixROI0)
        save_calib.write('validPixROI1', self.validPixROI1)
        save_calib.write('img_size', self.img_size)
        save_calib.release()


'''
board_size=(7, 7),
pattern=1,
physics_size=(16.4375, 15.875),
(7,5)
(36, 33.5)
'''

if __name__ == '__main__':
    file_path = r'D:\ubuntu18_win\rootfs\home\fq\lovely_robot\tools\fascinating_calib_stereo'
    file_strs = os.listdir(file_path)
    img0_str = [d for d in file_strs if 'main' in d]
    img1_str = [d for d in file_strs if 'aux' in d]
    img0_str = [os.path.join(file_path, d) for d in img0_str]
    img1_str = [os.path.join(file_path, d) for d in img1_str]
    imgs = [[], []]
    for idx, (file0_name, file1_name) in enumerate(zip(img0_str, img1_str)):
        img0 = cv2.imread(filename=file0_name)
        img1 = cv2.imread(filename=file1_name)
        imgs[0].append(img0)
        imgs[1].append(img1)

    # calib_main = LovelyCalibTool(imgs=[imgs[0]],
    #                              board_size=(7, 5),
    #                              pattern=1,
    #                              physics_size=(36, 33.5),
    #                              calib_save_to=file_path + '/tmp',
    #                              calib_save_name='calib_main.xml')
    # calib_main.Run()

    # calib_aux = LovelyCalibTool(imgs=[imgs[1]],
    #                             board_size=(7, 5),
    #                             pattern=1,
    #                             physics_size=(36, 33.5),
    #                             calib_save_to=file_path + '/tmp',
    #                             calib_save_name='calib_aux.xml')
    # calib_aux.Run()

    calib_stereo = CalibStereo(imgs=imgs,
                               board_size=(7, 5),
                               pattern=1,
                               physics_size=(36, 33.5),
                               calib_save_to=file_path)
    calib_stereo.Run()