# -*- coding: utf-8 -*-
import os
import json
import numpy as np
import platform
import cv2


class BaseScript:
    steering_engine_mode = ['#000PMOD!', '#001PMOD!', '#002PMOD!', '#003PMOD!', '#004PMOD!', '#005PMOD!']
    steering_release_mode = ['#000PULK!', '#001PULK!', '#002PULK!', '#003PULK!', '#004PULK!', '#005PULK!']
    steering_regain_mode = ['#000PULR!', '#001PULR!', '#002PULR!', '#003PULR!', '#004PULR!', '#005PULR!']
    steering_read_pos = [
        '#000PRAD!',
        '#001PRAD!',
        '#002PRAD!',
        '#003PRAD!',
        '#004PRAD!',
        '#005PRAD!',
    ]
    steering_straighten = '#000P1500T3000!#001P1500T3000!#002P1500T3000!#003P1500T3000!#004P1500T3000!#005P1500T3000!'
    steering_half_straighten = '#000P1496T3000!#001P2294T3000!#002P2255T3000!#003P1936T3000!#004P1477T3000!'
    steering_pos = '#000P1498T2000!#001P1797T2000!#002P2170T2000!#003P2061T2000!#004P1498T2000!'
    # for |    3.7V    |  7.4V
    #     | 100 | 4.2  |  8.4
    #     |  90 | 4.08 |  8.16
    #     |  80 | 4    |  8
    #     |  70 | 3.93 |  7.86
    #     |  60 | 3.87 |  7.74
    #     |  50 | 3.82 |  7.64
    #     |  40 | 3.79 |  7.58
    #     |  30 | 3.77 |  7.54
    #     |  20 | 3.73 |  7.46
    #     |  15 | 3.7  |  7.4
    #     |  10 | 3.68 |  7.36
    #     |   5 | 3.5  |  7
    #     |   0 | 2.5  |  5
    # cause control circuit made some limit, if the voltage is below 7 volts, it may lake of electricity
    # probably low on energy
    read_power_left = '#001PRTV!'

    init_set_power_on_release = '#000PCSM!#001PCSM!#002PCSM!#003PCSM!#004PCSM!#005PCSM!'
    init_set_steering_mode = '#000PMOD1!#001PMOD3!#002PMOD1!#003PMOD1!#004PMOD1!#005PMOD3!'

    cam_mode_single_cam_id = 0
    cam_mode_stereo_cam_id = (0, 2)
    cam_mode_stereo_cam_size = (1920, 1080)
    cam_open_mode = cv2.CAP_DSHOW


class BaseConfig:
    calib_path = './fascinating_calib_stereo/calib_stereo.xml'


base_script = BaseScript()
base_config = BaseConfig()