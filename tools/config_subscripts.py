# -*- coding: utf-8 -*-
import os
import json
import numpy as np
import platform


class BaseConfig:
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
    steering_half_straighten = '#001P1904T3000!#002P2301T3000!#003P1169T3000!#004P1467T3000!#005P1497!T3000!'
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
    init_set_steering_mode = '#000PMOD1!#001PMOD3!'
    
    cam_mode_single_cam_id = 1
    cam_mode_stereo_cam_id = (1, 2)


base_script = BaseConfig()