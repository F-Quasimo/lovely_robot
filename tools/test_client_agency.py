#!/usr/bin/python3
# 文件名：client.py

# 导入 socket、sys 模块
import socket
import sys
from server_agency import RoboClient
import cv2
from config_subscripts import base_config, base_script
from camera_mode import SingleCam
if __name__ == "__main__":
    capture = SingleCam(base_config.cam_mode_single_cam_id)
    capture.OpenCam()
    robo_client = RoboClient(host='192.168.43.163', port=8888)
    frame_count = 0
    while True:
        frame=capture.SnapShoot()
        if frame is None:
            continue
        print('client: ', frame_count)
        robo_client.Send(frame=frame, frame_idx=frame_count)
        feed_back = robo_client.Recv(frame_count)
        if feed_back != None:
            print(str(frame_count), feed_back)
        print('client: loop end ', frame_count)
        frame_count = frame_count + 1