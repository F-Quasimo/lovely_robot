# -*- coding: utf-8 -*-
import os
import cv2

class Stereo3D:
    def __init__(self, calib_file_path, **kwargs):
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
            calib_file_fs = cv2.FileStorage(calib_file_path, cv2.FILE_STORAGE_READ)
            self.camera_matrix_0 = calib_file_fs.getNode('cameraMatrix_0').mat()
            self.dist_coeffs_0 = calib_file_fs.getNode('distCoeffs_0').mat()
            self.optimal_matrix_0 = calib_file_fs.getNode('optimal_matrix_0').mat()
            self.camera_matrix_1 = calib_file_fs.getNode('cameraMatrix_1').mat()
            self.dist_coeffs_1 = calib_file_fs.getNode('distCoeffs_1').mat()
            self.optimal_matrix_1 = calib_file_fs.getNode('optimal_matrix_1').mat()

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
    
    def Rectify(self):
        #TBD
        return
    
    def StereoMatching(self):
        # TBD
        return
    
    def ReProjection(self):
        # TBD
        return
    
    def SaveImgAnd3D(self):
        # TBD
        return
    
    
    
