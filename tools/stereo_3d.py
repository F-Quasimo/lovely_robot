# -*- coding: utf-8 -*-
import os
import cv2
import numpy as np
import open3d as o3d
from nn_model.GMA.evaluate_single import GmaFlow
import time


class Stereo3D:
    def __init__(self, calib_file_path, actual_img_size=None, **kwargs):
        self.camera_matrix_0 = None
        self.dist_coeffs_0 = None
        self.optimal_matrix_0 = None
        self.camera_matrix_1 = None
        self.dist_coeffs_1 = None
        self.optimal_matrix_1 = None

        self.camera_matrix0 = None
        self.dist_coeffs0 = None
        self.camera_matrix1 = None
        self.dist_coeffs1 = None
        self.R_mat = None
        self.T_mat = None
        self.E_mat = None
        self.F_mat = None
        self.cam0_rectify = None
        self.cam1_rectify = None
        self.cam0_proj = None
        self.cam1_proj = None
        self.cam_Q_mat = None
        self.valid_pix_roi0 = None
        self.valid_pix_roi1 = None
        self.image_size = None

        self.cam0_mapx = None
        self.cam0_mapy = None
        self.cam1_mapx = None
        self.cam1_mapy = None

        def _load_calib_data():
            import sys

            print(sys.path[0])

            calib_file_fs = cv2.FileStorage(
                calib_file_path, cv2.FILE_STORAGE_READ)
            self.camera_matrix_0 = calib_file_fs.getNode(
                'cameraMatrix_0').mat()
            self.dist_coeffs_0 = calib_file_fs.getNode('distCoeffs_0').mat()
            self.optimal_matrix_0 = calib_file_fs.getNode(
                'optimal_matrix_0').mat()
            self.camera_matrix_1 = calib_file_fs.getNode(
                'cameraMatrix_1').mat()
            self.dist_coeffs_1 = calib_file_fs.getNode('distCoeffs_1').mat()
            self.optimal_matrix_1 = calib_file_fs.getNode(
                'optimal_matrix_1').mat()

            self.camera_matrix0 = calib_file_fs.getNode('cameraMatrix0').mat()
            self.dist_coeffs0 = calib_file_fs.getNode('distCoeffs0').mat()
            self.camera_matrix1 = calib_file_fs.getNode('cameraMatrix1').mat()
            self.dist_coeffs1 = calib_file_fs.getNode('distCoeffs1').mat()
            self.R_mat = calib_file_fs.getNode('R_mat').mat()
            self.T_mat = calib_file_fs.getNode('T_mat').mat()
            self.E_mat = calib_file_fs.getNode('E_mat').mat()
            self.F_mat = calib_file_fs.getNode('F_mat').mat()
            self.cam0_rectify = calib_file_fs.getNode('cam0_rectify').mat()
            self.cam1_rectify = calib_file_fs.getNode('cam1_rectify').mat()
            self.cam0_proj = calib_file_fs.getNode('cam0_proj').mat()
            self.cam1_proj = calib_file_fs.getNode('cam1_proj').mat()
            self.cam_Q_mat = calib_file_fs.getNode('cam_Q_mat').mat()
            self.valid_pix_roi0 = calib_file_fs.getNode('validPixROI0').mat()
            self.valid_pix_roi1 = calib_file_fs.getNode('validPixROI1').mat()
            self.image_size = calib_file_fs.getNode('img_size').mat()

        _load_calib_data()

        scales_k = -1
        if actual_img_size is not None:
            scales_k = actual_img_size[0]/self.image_size[0]

        self.cam0_mapx, self.cam0_mapy = cv2.initUndistortRectifyMap(cameraMatrix=self.camera_matrix0,
                                                                     distCoeffs=self.dist_coeffs0,
                                                                     R=self.cam0_rectify,
                                                                     newCameraMatrix=self.cam0_proj,
                                                                     size=(int(self.image_size[0]),
                                                                           int(self.image_size[1])),
                                                                     m1type=cv2.CV_32FC1)
        self.cam1_mapx, self.cam1_mapy = cv2.initUndistortRectifyMap(cameraMatrix=self.camera_matrix1,
                                                                     distCoeffs=self.dist_coeffs1,
                                                                     R=self.cam1_rectify,
                                                                     newCameraMatrix=self.cam1_proj,
                                                                     size=(int(self.image_size[0]),
                                                                           int(self.image_size[1])),
                                                                     m1type=cv2.CV_32FC1)
        if scales_k > 0:
            self.cam0_mapx = (cv2.resize(
                self.cam0_mapx, dsize=actual_img_size)*scales_k).astype('float32')
            self.cam0_mapy = (cv2.resize(
                self.cam0_mapy, dsize=actual_img_size)*scales_k).astype('float32')
            self.cam1_mapx = (cv2.resize(
                self.cam1_mapx, dsize=actual_img_size)*scales_k).astype('float32')
            self.cam1_mapy = (cv2.resize(
                self.cam1_mapy, dsize=actual_img_size)*scales_k).astype('float32')
            scales = np.zeros((3, 3), dtype='float32')
            # scales[0,0] =
            self.cam_Q_mat[:, 3] = self.cam_Q_mat[:, 3]*scales_k

        self.mapped_img0 = None
        self.mapped_img1 = None
        self.disparity = None
        self.flow = None
        self.model = None
        self.img_3d = None

    def Rectify(self, img0, img1, write_down=False):
        self.mapped_img0 = cv2.remap(
            img0, map1=self.cam0_mapx, map2=self.cam0_mapy, interpolation=cv2.INTER_AREA)
        self.mapped_img1 = cv2.remap(
            img1, map1=self.cam1_mapx, map2=self.cam1_mapy, interpolation=cv2.INTER_AREA)
        if write_down:
            combine = np.hstack([self.mapped_img0, self.mapped_img1])
            for line_idx in range(20):
                combine = cv2.line(combine, (0, int(self.image_size[1] / 20 * line_idx)),
                                   (int(
                                       self.image_size[0] * 2), int(self.image_size[1] / 20 * line_idx)), (0, 0, 255),
                                   2, cv2.LINE_4)
            cv2.imwrite(os.path.join('./', 'combine_' +
                        str('test') + '.jpg'), combine)
        return

    def StereoMatching(self, write_down=False, return_8u=False, method='sgbm', **kwargs):
        if method == 'sgbm':
            window_size = 11
            stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=240,  # max_disp has to be dividable by 16 f. E. HH 192, 256
                blockSize=7,
                P1=1 * 3 * window_size**2,
                # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
                P2=32 * 3 * window_size**2,
                disp12MaxDiff=5,
                uniquenessRatio=15,
                speckleWindowSize=200,
                speckleRange=2,
                preFilterCap=63,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
            disparity = stereo.compute(
                self.mapped_img0, self.mapped_img1).astype(np.float32) / 16.0
            self.disparity = disparity
        elif method == 'GMA':
            if self.model == None:
                self.model = GmaFlow()
            optical_flow = self.model.Run(
                img0=self.mapped_img0, img1=self.mapped_img1, iters=kwargs['iter'])
            # print('kwargs[iter]',kwargs['iter'])
            self.disparity = optical_flow[:, :, 0] * -1
        # flow
        # optical_flow_gen = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)
        # tar_8u = cv2.cvtColor(self.mapped_img0, cv2.COLOR_BGR2GRAY)
        # curr_8u = cv2.cvtColor(self.mapped_img1, cv2.COLOR_BGR2GRAY)
        # optical_flow_0 = optical_flow_gen.calc(tar_8u, curr_8u, None)
        # flow = optical_flow_0.copy()
        # self.disparity = flow[:,:,0]*-1
        disparity_u8 = None
        if write_down or return_8u:

            disparity_u8 = cv2.normalize(
                src=self.disparity, dst=disparity_u8, norm_type=cv2.NORM_MINMAX)
            disparity_u8 = (disparity_u8 * 255).astype(np.uint8)
            if write_down:
                cv2.imwrite(os.path.join('./', 'disp_' +
                            str('test') + '.jpg'), disparity_u8)
        if return_8u:
            return disparity_u8
        else:
            return None

    def ReProjection(self, write_down=False):
        _3d_image = cv2.reprojectImageTo3D(
            disparity=self.disparity, Q=self.cam_Q_mat, handleMissingValues=True)
        xyz = _3d_image.reshape(-1, 3)
        xyz = xyz[xyz[:, 2] < 1000]
        xyz = xyz[xyz[:, 2] > -100]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        self.img_3d = pcd
        if write_down:
            o3d.io.write_point_cloud("./test_sync.ply", pcd)
        return

    def SaveImgAnd3D(self):
        # TBD
        return


if __name__ == '__main__':
    # import sys
    # import argparse
    # import os
    # sys.path.append('.')
    actual_img_size = (640, 360)
    stereo_3d = Stereo3D(
        calib_file_path='calib_stereo/calib_stereo.xml', actual_img_size=actual_img_size)
    img0 = cv2.imread('./stereo_test/p_main_0000.jpg')
    img1 = cv2.imread('./stereo_test/p_aux_0000.jpg')
    img0 = cv2.resize(img0, actual_img_size)
    img1 = cv2.resize(img1, actual_img_size)
    stereo_3d.Rectify(img0=img0, img1=img1, write_down=True)
    time_before = time.time()
    dic = {'iter': 5}
    stereo_3d.StereoMatching(write_down=True, method='GMA', **dic)
    time_after = time.time()
    print('Matching cost: ', time_after-time_before)
    stereo_3d.ReProjection(write_down=True)
