# -*- coding: utf-8 -*-
import os
import re
import cv2
import numpy as np
import platform
import threading
import time
from stereo_3d import Stereo3D
from config_subscripts import base_config, base_script


class SingleCam:
    def __init__(self, cam_id=0, cam_size=(1920, 1080), cam_mode=cv2.CAP_DSHOW, cam_fps=30, exposure=-6, bright=0):
        self.cam_id = cam_id
        self.cap = None
        self.cam_size = cam_size
        self.cap_open_mode = cam_mode
        self.cam_fps = cam_fps
        self.exposure = exposure
        self.bright = bright

        self.frame_buffer = [None, None]
        self.frame_idx = 0
        self.thread_alive = False
        self.videocap_thread = None

    def _thread_cam(self):
        while self.thread_alive:
            ret, snap = self.cap.read()
            self.frame_buffer[self.frame_idx] = snap
            self.frame_idx = (self.frame_idx + 1) % 2
            # print('DEBUG FPS: ', self.cap.get(cv2.CAP_PROP_FPS), ' exp ', self.cap.get(cv2.CAP_PROP_EXPOSURE),
            #       'frame_size: ', self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT), ' ', self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))

    def OpenCam(self, time_sleep=0):
        try:
            self.Close()
        except:
            print('CLOSE IN OpenCam ERROR')
            return False
        if time_sleep > 0:
            time.sleep(time_sleep)
        print('OpenCam: Single ', self.cam_id, ' mode: ', self.cap_open_mode)
        self.cap = cv2.VideoCapture(self.cam_id + self.cap_open_mode)
        self.cap.open(self.cam_id)
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_size[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.cam_fps)
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,1)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE,-4)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        print('DEBUG SingleCam OpenCam')
        if self.cap.isOpened():
            self.thread_alive = True
            self.videocap_thread = threading.Thread(target=self._thread_cam)
            self.videocap_thread.start()
            return True
        else:
            self.cap = None
            print('DEBUG SingleCam OpenCam FAIL')
            return False

    def SnapShoot(self):
        if self.thread_alive:
            snap = self.frame_buffer[(self.frame_idx + 1) % 2]
            while snap is None:
                snap = self.frame_buffer[(self.frame_idx + 1) % 2]
            return snap
        else:
            return None

    def Close(self):
        self.thread_alive = False
        if self.cap is None:
            return
        if self.cap.isOpened():
            self.cap.release()
            self.cap = None


class StereoCam:
    def __init__(self,
                 cam_id0,
                 cam_id1,
                 cam_size=(1920, 1080),
                 cam_mode=cv2.CAP_DSHOW,
                 cam_fps=30,
                 exposure=-6,
                 bright=0):
        self.cam_id0 = cam_id0
        self.cam_id1 = cam_id1
        self.cap0 = None
        self.cap1 = None
        self.cam_size = cam_size
        self.cap_open_mode = cam_mode
        self.cam_fps = cam_fps
        self.exposure = exposure
        self.bright = bright

        self.frame_buffer = [None, None, None, None]
        self.frame_idx = 0
        self.thread_alive = False
        self.videocap_thread = None

    def _thread_cam(self):
        while self.thread_alive:
            ret, snap0 = self.cap0.read()
            ret, snap1 = self.cap1.read()
            self.frame_buffer[self.frame_idx] = snap0
            self.frame_buffer[self.frame_idx + 1] = snap1
            self.frame_idx = (self.frame_idx + 2) % 4
            # print('DEBUG FPS: ',
            #       self.cap0.get(cv2.CAP_PROP_FPS), ' brigh ', self.cap0.get(cv2.CAP_PROP_BRIGHTNESS), ' exp ',
            #       self.cap0.get(cv2.CAP_PROP_EXPOSURE), 'frame_size: ', self.cap0.get(cv2.CAP_PROP_FRAME_HEIGHT), ' ',
            #       self.cap0.get(cv2.CAP_PROP_FRAME_WIDTH), 'DEBUG2 FPS: ', self.cap1.get(cv2.CAP_PROP_FPS), ' brigh ',
            #       self.cap1.get(cv2.CAP_PROP_BRIGHTNESS), ' exp ', self.cap1.get(cv2.CAP_PROP_EXPOSURE), 'frame_size: ',
            #       self.cap1.get(cv2.CAP_PROP_FRAME_HEIGHT), ' ', self.cap1.get(cv2.CAP_PROP_FRAME_WIDTH), ' iso ',
            #       self.cap1.get(cv2.CAP_PROP_ISO_SPEED))

    def OpenCam(self, time_sleep=0):
        try:
            # print('TRY CLOSE')
            self.Close()
            # print('STEREO TRY CLOSE OK')
        except:
            print('CLOSE IN OpenCam ERROR')
            return False
        if time_sleep > 0:
            # print('SLEEP START')
            time.sleep(time_sleep)
            # print('SLEEP OVER')
        self.cap0 = cv2.VideoCapture(self.cam_id0 + self.cap_open_mode)
        self.cap0.open(self.cam_id0)
        self.cap0.set(cv2.CAP_PROP_FOURCC,
                      cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap0.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_size[0])
        self.cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_size[1])
        self.cap0.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)
        self.cap0.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        self.cap0.set(cv2.CAP_PROP_FPS, self.cam_fps)
        self.cap1 = cv2.VideoCapture(self.cam_id1 + self.cap_open_mode)
        self.cap1.open(self.cam_id1)
        self.cap1.set(cv2.CAP_PROP_FOURCC,
                      cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_size[0])
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_size[1])
        self.cap1.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)
        self.cap1.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        self.cap1.set(cv2.CAP_PROP_FPS, self.cam_fps)
        if self.cap0.isOpened() and self.cap1.isOpened():
            self.thread_alive = True
            self.videocap_thread = threading.Thread(target=self._thread_cam)
            self.videocap_thread.start()
            return True
        else:
            self.Close()
            return False

    def SnapShoot(self):
        if self.thread_alive:
            snap0 = self.frame_buffer[(self.frame_idx + 2) % 4]
            snap1 = self.frame_buffer[(self.frame_idx + 3) % 4]
            while snap0 is None or snap1 is None:
                snap0 = self.frame_buffer[(self.frame_idx + 2) % 4]
                snap1 = self.frame_buffer[(self.frame_idx + 3) % 4]
            return snap0, snap1
        else:
            return None, None

    def Close(self):
        self.thread_alive = False
        if self.cap0 is not None:
            if self.cap0.isOpened():
                self.cap0.release()
                self.cap0 = None
        if self.cap1 is not None:
            if self.cap1.isOpened():
                self.cap1.release()
                self.cap1 = None


if __name__ == '__main__':
    INFERENCE = 'Person'
    from nn_model.yolox.inference import YoloX
    sigcam = SingleCam(cam_id=0, cam_size=(1920, 1080), cam_mode=cv2.CAP_DSHOW, cam_fps=30,
                       bright=20,
                       exposure=400)
    sigcam.OpenCam()
    yolox = YoloX(
        pt_path='/home/fq/lovely_robot/tools/nn_model/yolox/checkpoints/yolox_nano_416416_torchscript.pt')
    from nn_model.simple_pose.simple_pose import PersonDetect
    person_detect = PersonDetect(param_path=base_config.person_detect_param_path,
                                 bin_path=base_config.person_detect_bin_path)
    from nn_model.simple_pose.simple_pose import SimplePose, SimplePose2
    simple_pose = SimplePose2(param_path=base_config.simple_pose2_param_path,
                              bin_path=base_config.simple_pose2_bin_path)
    # simple_pose = SimplePose(param_path='/home/fq/lovely_robot/tools/nn_model/simple_pose/checkpoints/Ultralight-Nano-SimplePose.param',
    # bin_path='/home/fq/lovely_robot/tools/nn_model/simple_pose/checkpoints/Ultralight-Nano-SimplePose.bin')
    while True:
        if not sigcam.cap.isOpened():
            cv2.waitKey(3)
            print('wait...\n')
            continue
        snap0 = sigcam.SnapShoot()
        snap0 = cv2.rotate(snap0, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if snap0 is None:
            print('NONE OF IMG')
            continue
        if INFERENCE == 'yolox':
            time_before = time.time()
            snap0 = cv2.resize(snap0, (416, 234))
            output, img_info = yolox.inference(img_in=snap0.copy())
            vis_ans = yolox.visual(
                output=output[0], img_info=img_info, cls_conf=0.6)
            time_end = time.time()
            print('fps: ', 1/(time_end-time_before))
            cv2.imshow('yolox_out', vis_ans)
        elif INFERENCE == 'Person':
            time_before = time.time()
            snap0_320 = cv2.resize(snap0, (320, 320))
            result = person_detect(snap0.copy())
            time_end = time.time()
            print('fps: ', 1/(time_end-time_before))
            if result is not None:
                cv2.imshow('ncnn0', result)
                # if result.shape[0]/result.shape[1] > 256/192:
                #     h=
                result = cv2.resize(result, (192, 256))
                # result=cv2.imread('/home/fq/github/Lite-HRNet/resources/cxk256192.png')
                kps, result = simple_pose(result)
                if result is not None:
                    cv2.imshow('ncnn1', cv2.resize(result, (600, 800)))
        cv2.imshow('snap0', snap0)

        cv2.waitKey(1)
        #cv2.imwrite('/home/pi/github/save.jpg', snap0)
    '''
    stereo_cam = StereoCam(cam_id0=base_config.cam_mode_stereo_cam_id[0],
                           cam_id1=base_config.cam_mode_stereo_cam_id[1],
                           cam_size=(640, 360),
                           cam_mode=cv2.CAP_DSHOW,
                           cam_fps=30,
                           bright=0,
                           exposure=500)
    stereo_cam.OpenCam()
    stereo_3d = Stereo3D(calib_file_path='./calib_stereo/calib_stereo.xml',
                         actual_img_size=(512, 288))
    while True:
        if not stereo_cam.cap1.isOpened():
            cv2.waitKey(3)
            print('wait...\n')
            continue
        snap0, snap1 = stereo_cam.SnapShoot()
        snap0 = cv2.resize(snap0, (512, 288))
        snap1 = cv2.resize(snap1, (512, 288))

        stereo_3d.Rectify(img0=snap0, img1=snap1, write_down=False)
        dic = {'iter': 5}
        return_8u = stereo_3d.StereoMatching(
            write_down=False, return_8u=True, method='GMA', **dic)
        cv2.imshow('snap0', snap0)
        cv2.imshow('snap1', snap1)
        cv2.imshow('disp', return_8u)
        cv2.waitKey(1)
        # cv2.imwrite('/home/pi/github/save00.jpg', snap0)
        # cv2.imwrite('/home/pi/github/save01.jpg', snap1)
    '''
