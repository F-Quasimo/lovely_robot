# -*- coding: utf-8 -*-
from cmath import pi
from ctypes import alignment
import os
from textwrap import fill
import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
import numpy as np
import shutil
import serial
import serial.tools.list_ports
import threading
import inspect
import time
import ctypes
from config_subscripts import base_script, base_config
from config_fasci import fasci_config
from camera_mode import SingleCam, StereoCam
from motion_act import RobotArm
from camera_calib import LovelyCalibTool, CalibStereo
from wheels import Wheels
from status_control import MotionStatus
from robo_serial import RoboSerial


class TuningGUI:
    def __init__(self):
        tk_tittle = 'tuning_tools'
        self.window = tk.Tk()
        self.window.title(tk_tittle)
        self.frame_main = self.window

        # *************************** app run var *********************
        self.my_serial = None
        self.my_serial_prefer = base_config.serial_prefer
        self.command_send_buffer = ''
        # single camera \ stereo_camera \ single_track \ stereo_left_track \ stereo_3d_track
        self.cam_mode = ['SingleCam', 'Stereo',
                         '1_Track', '1_L_Track', '3DTrack']
        self.cam_mode_flag = 0
        self.cam_mode_thread = [None] * len(self.cam_mode)
        self.cam_mode_thread_flag = [False] * len(self.cam_mode)
        self.thread_single_flag = 0
        self.thread_stereo_flag = 1
        self.thread_1_track_flag = 2
        self.thread_1_l_track_flag = 3
        self.thread_3d_track_flag = 4

        # click for grap video stream right click for snap
        self.cap_curr_snap = False
        self.cap_tar_snap = False
        self.cap_curr_snapped = False
        self.cap_tar_snapped = False
        self.cam_mode_single_cam_id = base_config.cam_mode_single_cam_id
        self.cam_mode_stereo_cam_id = base_config.cam_mode_stereo_cam_id
        self.cam_mode_stereo_cam_size = base_config.cam_mode_stereo_cam_size
        self.cam_open_mode = base_config.cam_open_mode

        # **** Read Calib single camera
        fs_read_calib = cv2.FileStorage(
            base_config.calib_path, cv2.FileStorage_READ)
        self.camera_matrix = fs_read_calib.getNode('cameraMatrix_0').mat()
        self.optimal_matrix = fs_read_calib.getNode('optimal_matrix_0').mat()
        self.distortion = fs_read_calib.getNode('distCoeffs_0').mat()
        fs_read_calib.release()

        # define robot arm
        self.robot_arm = RobotArm()

        # define foot
        self.foot = Wheels()
        self.foot_base_speed = 500
        self.foot_base_time = 20
        self.motion_status = MotionStatus(self.foot)
        # ***** for calib
        # 0 for default ,
        self.calib_step = 0
        # config step 1
        self.calib_camera_id = 0
        self.calib_camera_type = 'single'
        # config step 2
        self.calib_img_size = (1920, 1080)
        self.calib_imgs = [[]]
        # config step 3
        self.calib_board_size = (7, 7)
        # square config step 4
        self.calib_pattern = 1
        # config step 5
        self.calib_physics_size = (16.4375, 15.875)
        # config step 6
        self.calib_save_to = '.'
        self.calib_cam_main_name = 'p_main_'
        self.calib_cam_aux_name = 'p_aux_'
        self.calib_save_name = 'calib.xml'
        self.calib_cam_open_mode = cv2.CAP_DSHOW
        self.calib_cam_cap_thread = None
        self.calib_snap_count = 0
        # step 7 start camera stream step 8: take calib imgs
        self.calib_main_cameraMatrix = None
        self.calib_main_distCoeffs = None
        self.calib_main_optimal_matrix = None
        self.calib_aux_cameraMatrix = None
        self.calib_aux_distCoeffs = None
        self.calib_aux_optimal_matrix = None

        # **************************************************************
        self.monitor = tk.PanedWindow(self.frame_main, orient=tk.VERTICAL)
        self.euler_show = tk.PanedWindow(self.frame_main, orient=tk.VERTICAL)

        self.type_in = tk.PanedWindow(self.frame_main, orient=tk.VERTICAL)

        self.buttons_and_serial = tk.Frame(self.frame_main)
        self.buttons = tk.PanedWindow(
            self.buttons_and_serial, orient=tk.VERTICAL)
        self.serial_com = tk.PanedWindow(
            self.buttons_and_serial, orient=tk.VERTICAL)

        self.frame_1 = tk.LabelFrame(
            master=self.monitor, bg='#555555555', text='monitor')
        self.frame_2 = tk.LabelFrame(
            master=self.euler_show, bg='#555555555', text='euler_show')
        self.frame_3 = tk.LabelFrame(
            master=self.buttons, bg='#555555555', text='control')
        self.frame_4 = tk.LabelFrame(
            master=self.type_in, bg='#555555555', text='input')
        self.frame_5 = tk.LabelFrame(
            master=self.serial_com, bg='#555555555', text='serial')

        self.frame_2.pack(expand='YES')

        # self.monitor.add(self.frame_1)
        # self.monitor.grid(row=0,column=0)
        # self.euler_show.add(self.frame_2)
        # # self.euler_show.columnconfigure(0, weight=1)
        # # self.euler_show.rowconfigure(0, weight=1)
        # self.euler_show.grid(row=1, column=0, sticky=tk.NSEW, ipadx=1,ipady=1, padx=1, pady=1)
        # self.buttons.add(self.frame_3)
        # self.buttons.grid(row=2,column=0)
        # self.type_in.add(self.frame_4)
        # self.type_in.grid(row=3,column=0)
        # self.serial_com.add(self.frame_5)
        # self.serial_com.grid(row=4,column=0)
        self.monitor.add(self.frame_1)
        self.monitor.pack()
        self.euler_show.add(self.frame_2)
        # self.euler_show.columnconfigure(0, weight=1)
        # self.euler_show.rowconfigure(0, weight=1)
        self.euler_show.pack()
        self.type_in.add(self.frame_4)
        self.type_in.pack()
        self.buttons.add(self.frame_3)
        self.buttons.pack(side=tk.LEFT)
        self.serial_com.add(self.frame_5)
        self.serial_com.pack(side=tk.LEFT)
        self.buttons_and_serial.pack()

        # ***************** MONITOR ******************
        empty_img = np.ones((384, 512, 3), dtype='uint8') * 255
        self.pic_height = 384
        self.pic_width = 512
        self.w_div_h = self.pic_width / self.pic_height
        self.np_tar = empty_img.copy().astype(np.uint8)
        self.image_tar = Image.fromarray(self.np_tar)
        self.monitor_tar_img = ImageTk.PhotoImage(self.image_tar)
        self.monitor_tar = tk.Label(master=self.frame_1,
                                    image=self.monitor_tar_img,
                                    height=self.pic_height,
                                    width=self.pic_width)
        self.monitor_tar.pack(side=tk.LEFT)

        self.np_curr = empty_img.copy().astype(np.uint8)
        self.image_curr = Image.fromarray(self.np_curr)
        self.monitor_curr_img = ImageTk.PhotoImage(self.image_curr)
        self.monitor_curr = tk.Canvas(
            self.frame_1, bg='#345645323', width=self.pic_width, height=self.pic_height)
        self.monitor_curr = tk.Label(master=self.frame_1,
                                     image=self.monitor_curr_img,
                                     height=self.pic_height,
                                     width=self.pic_width)
        self.monitor_curr.pack(side=tk.LEFT)

        self.np_show = empty_img.copy().astype(np.uint8)
        self.image_show = Image.fromarray(self.np_show)
        self.monitor_show_img = ImageTk.PhotoImage(self.image_show)
        self.monitor_show = tk.Canvas(
            self.frame_1, bg='#345645323', width=self.pic_width, height=self.pic_height)
        self.monitor_show = tk.Label(master=self.frame_1,
                                     image=self.monitor_show_img,
                                     height=self.pic_height,
                                     width=self.pic_width)
        self.monitor_show.pack(side=tk.LEFT)

        # ********************** EULER_SHOW *********************
        font_height = 2
        font_width = 34
        euler_show_padx = 4
        self.pitch = tk.Label(self.frame_2, text='Pitch_x',
                              height=font_height, width=font_width, bg='#000fff000')
        self.pitch.grid(row=0, column=0, sticky=tk.NSEW,
                        padx=euler_show_padx, pady=1, ipadx=1, ipady=1)
        self.yaw = tk.Label(self.frame_2, text='Yaw_y',
                            height=font_height, width=font_width, bg='#000fff000')
        self.yaw.grid(row=0, column=1, sticky=tk.NSEW,
                      padx=euler_show_padx, pady=1)
        self.roll = tk.Label(self.frame_2, text='Roll_z',
                             height=font_height, width=font_width, bg='#000fff000')
        self.roll.grid(row=0, column=2, sticky=tk.NSEW,
                       padx=euler_show_padx, pady=1)
        self.shift_x = tk.Label(self.frame_2, text='Shift_x',
                                height=font_height, width=font_width, bg='#000fff000')
        self.shift_x.grid(row=0, column=3, sticky=tk.NSEW,
                          padx=euler_show_padx, pady=1)
        self.shift_y = tk.Label(self.frame_2, text='Shift_y',
                                height=font_height, width=font_width, bg='#000fff000')
        self.shift_y.grid(row=0, column=4, sticky=tk.NSEW,
                          padx=euler_show_padx, pady=1)
        self.shift_z = tk.Label(self.frame_2, text='Shift_z',
                                height=font_height, width=font_width, bg='#000fff000')
        self.shift_z.grid(row=0, column=5, sticky=tk.NSEW,
                          padx=euler_show_padx, pady=1)
        # ------
        font_height_val = 2
        self.pitch_val = tk.Label(
            self.frame_2, text='0.00', height=font_height_val)
        self.pitch_val.grid(row=1, column=0, sticky=tk.NSEW,
                            padx=euler_show_padx, pady=1)
        self.yaw_val = tk.Label(
            self.frame_2, text='0.00', height=font_height_val)
        self.yaw_val.grid(row=1, column=1, sticky=tk.NSEW,
                          padx=euler_show_padx, pady=1)
        self.roll_val = tk.Label(
            self.frame_2, text='0.00', height=font_height_val)
        self.roll_val.grid(row=1, column=2, sticky=tk.NSEW,
                           padx=euler_show_padx, pady=1)
        self.shift_x_val = tk.Label(
            self.frame_2, text='0.00', height=font_height_val)
        self.shift_x_val.grid(row=1, column=3, sticky=tk.NSEW,
                              padx=euler_show_padx, pady=1)
        self.shift_y_val = tk.Label(
            self.frame_2, text='0.00', height=font_height_val)
        self.shift_y_val.grid(row=1, column=4, sticky=tk.NSEW,
                              padx=euler_show_padx, pady=1)
        self.shift_z_val = tk.Label(
            self.frame_2, text='0.00', height=font_height_val)
        self.shift_z_val.grid(row=1, column=5, sticky=tk.NSEW,
                              padx=euler_show_padx, pady=1)

        # ******************* TYPE IN *********************
        self.json_label = tk.Label(self.frame_4, text='Json:', height=3)
        path_default = tk.StringVar(
            value='/home/fq/lovely_robot/script/init_script.json')
        self.json_path = tk.Entry(self.frame_4, font=(
            'Arial', 16), width=55, textvariable=path_default)
        self.send_label = tk.Label(self.frame_4, text='Send:', height=3)
        self.send_command_entry = tk.Entry(
            self.frame_4, font=('Arial', 16), width=65)
        self.json_label.pack(side=tk.LEFT, padx=3, pady=3)
        self.json_path.pack(side=tk.LEFT)
        self.send_label.pack(side=tk.LEFT, padx=3, pady=3)
        self.send_command_entry.pack(side=tk.LEFT)

        # *********************** BUTTON *****************
        button_font = ('Arial', 12)
        button_w = 9
        button_padx = 7
        button_pady = 7
        delta_move = 4
        delta_mini_move = 0.5
        self.pitch_p_bt = tk.Button(
            self.frame_3, text='Pitch+^', font=button_font, width=button_w)
        self.pitch_p_bt.bind('<Button-1>',
                             lambda event: self._send_command(self.motion_status.PitchUp(delta_x=delta_move, time_t=2000)))
        self.pitch_p_bt.bind(
            '<Button-3>',
            lambda event: self._send_command(self.motion_status.PitchUp(delta_x=delta_mini_move, time_t=2000)))
        self.pitch_p_bt.grid(row=0, column=0, sticky=tk.NSEW,
                             padx=button_padx, pady=button_pady)

        self.yaw_p_bt = tk.Button(
            self.frame_3, text='Yaw+>', font=button_font, width=button_w)
        self.yaw_p_bt.bind('<Button-1>',
                           lambda event: self._send_command(self.motion_status.JointYawUp(delta_x=delta_move, time_t=2000)))
        self.yaw_p_bt.bind(
            '<Button-3>',
            lambda event: self._send_command(self.motion_status.JointYawUp(delta_x=delta_mini_move, time_t=2000)))
        self.yaw_p_bt.grid(row=0, column=1, sticky=tk.NSEW,
                           padx=button_padx, pady=button_pady)

        self.roll_p_bt = tk.Button(
            self.frame_3, text='Roll+@', font=button_font, width=button_w)
        self.roll_p_bt.bind('<Button-1>',
                            lambda event: self._send_command(self.motion_status.RollUp(delta_x=delta_move, time_t=2000)))
        self.roll_p_bt.bind(
            '<Button-3>',
            lambda event: self._send_command(self.motion_status.RollUp(delta_x=delta_mini_move, time_t=2000)))
        self.roll_p_bt.grid(row=0, column=2, sticky=tk.NSEW,
                            padx=button_padx, pady=button_pady)

        self.run_lf_bt = tk.Button(
            self.frame_3, text='LF', font=button_font, width=button_w)
        self.run_lf_bt.bind('<Button-1>', self._buttun_none)
        self.run_lf_bt.grid(row=0, column=3, sticky=tk.NSEW,
                            padx=button_padx, pady=button_pady)

        self.run_f_bt = tk.Button(
            self.frame_3, text='Forward ^', font=button_font, width=button_w)
        self.run_f_bt.bind(
            '<Button-1>',
            lambda event: self._send_command(self.foot.FrontTranslation(speed=self.foot_base_speed, time_run=self.foot_base_time)))
        self.run_f_bt.grid(row=0, column=4, sticky=tk.NSEW,
                           padx=button_padx, pady=button_pady)

        self.run_rf_bt = tk.Button(
            self.frame_3, text='RF', font=button_font, width=button_w)
        self.run_rf_bt.bind('<Button-1>', self._buttun_none)
        self.run_rf_bt.grid(row=0, column=5, sticky=tk.NSEW,
                            padx=button_padx, pady=button_pady)

        self.cap_tar_bt = tk.Button(
            self.frame_3, text='CapTar', font=button_font, width=button_w)
        self.cap_tar_bt.bind('<Button-1>', self._cap_tar_button_func)
        self.cap_tar_bt.bind('<Button-3>', self._cap_tar_button_right_func)
        self.cap_tar_bt.grid(row=0, column=6, sticky=tk.NSEW,
                             padx=button_padx, pady=button_pady)

        self.pitch_m_bt = tk.Button(
            self.frame_3, text='Pitch-v', font=button_font, width=button_w)
        self.pitch_m_bt.bind(
            '<Button-1>',
            lambda event: self._send_command(self.motion_status.PitchUp(delta_x=-1 * delta_move, time_t=2000)))
        self.pitch_m_bt.bind(
            '<Button-3>',
            lambda event: self._send_command(self.motion_status.PitchUp(delta_x=-1 * delta_mini_move, time_t=2000)))
        self.pitch_m_bt.grid(row=1, column=0, sticky=tk.NSEW,
                             padx=button_padx, pady=button_pady)

        self.yaw_m_bt = tk.Button(
            self.frame_3, text='Yaw-<', font=button_font, width=button_w)
        self.yaw_m_bt.bind('<Button-1>',
                           lambda event: self._send_command(self.motion_status.JointYawUp(delta_x=-1 * delta_move, time_t=2000)))
        self.yaw_m_bt.bind(
            '<Button-3>',
            lambda event: self._send_command(self.motion_status.JointYawUp(delta_x=-1 * delta_mini_move, time_t=2000)))
        self.yaw_m_bt.grid(row=1, column=1, sticky=tk.NSEW,
                           padx=button_padx, pady=button_pady)

        self.roll_m_bt = tk.Button(
            self.frame_3, text='Roll-G', font=button_font, width=button_w)
        self.roll_m_bt.bind(
            '<Button-1>',
            lambda event: self._send_command(self.motion_status.RollUp(delta_x=-1 * delta_move, time_t=2000)))
        self.roll_m_bt.bind(
            '<Button-3>',
            lambda event: self._send_command(self.motion_status.RollUp(delta_x=-1 * delta_mini_move, time_t=2000)))
        self.roll_m_bt.grid(row=1, column=2, sticky=tk.NSEW,
                            padx=button_padx, pady=button_pady)

        self.run_left_bt = tk.Button(
            self.frame_3, text='Left <', font=button_font, width=button_w)
        self.run_left_bt.bind('<Button-1>', lambda event: self._send_command(
            self.foot.LeftTranslation(speed=self.foot_base_speed, time_run=self.foot_base_time)))
        self.run_left_bt.grid(row=1, column=3, sticky=tk.NSEW,
                              padx=button_padx, pady=button_pady)

        self.run_pause_bt = tk.Button(
            self.frame_3, text='Run', font=button_font, width=button_w)
        self.run_pause_bt.bind('<Button-1>', lambda event: self._send_command(
            self.foot.FrontTranslation(speed=0, time_run=self.foot_base_time)))
        self.run_pause_bt.grid(
            row=1, column=4, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_right_bt = tk.Button(
            self.frame_3, text='Right >', font=button_font, width=button_w)
        self.run_right_bt.bind('<Button-1>', lambda event: self._send_command(
            self.foot.LeftTranslation(speed=-1*self.foot_base_speed, time_run=self.foot_base_time)))
        self.run_right_bt.grid(
            row=1, column=5, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.cap_curr_bt = tk.Button(
            self.frame_3, text='CapCurr', font=button_font, width=button_w)
        self.cap_curr_bt.bind('<Button-1>', self._cap_curr_button_func)
        self.cap_curr_bt.bind('<Button-3>', self._cap_curr_button_right_func)
        self.cap_curr_bt.grid(row=1, column=6, sticky=tk.NSEW,
                              padx=button_padx, pady=button_pady)

        # ******************* BUTTON ROW 2 *********************
        self.stand_up_bt = tk.Button(
            self.frame_3, text='StandUp~', font=button_font, width=button_w)
        self.stand_up_bt.bind('<Button-1>',
                              lambda event: self._send_command(self.motion_status.StandUp(delta_x=delta_move, time_t=2000)))
        self.stand_up_bt.bind(
            '<Button-3>',
            lambda event: self._send_command(self.motion_status.StandUp(delta_x=delta_mini_move, time_t=2000)))
        self.stand_up_bt.grid(row=2, column=0, sticky=tk.NSEW,
                              padx=button_padx, pady=button_pady)

        self.release_machine_bt = tk.Button(
            self.frame_3, text='Release', font=button_font, width=button_w)
        self.release_machine_bt.bind(
            '<Button-1>', lambda event: self._send_command(base_script.steering_release_mode))
        self.release_machine_bt.grid(
            row=2, column=1, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.regain_bt = tk.Button(
            self.frame_3, text='Regain', font=button_font, width=button_w)
        self.regain_bt.bind(
            '<Button-1>', lambda event: self._send_command(base_script.steering_regain_mode))
        self.regain_bt.grid(row=2, column=2, sticky=tk.NSEW,
                            padx=button_padx, pady=button_pady)

        self.run_l_circle_bt = tk.Button(
            self.frame_3, text='L_Circle', font=button_font, width=button_w)
        self.run_l_circle_bt.bind('<Button-1>', lambda event: self._send_command(
            self.foot.TurnLeft(speed=self.foot_base_speed, time_run=self.foot_base_time)))
        self.run_l_circle_bt.grid(
            row=2, column=3, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_retreat_bt = tk.Button(
            self.frame_3, text='Retreat v', font=button_font, width=button_w)
        self.run_retreat_bt.bind('<Button-1>',
                                 lambda event: self._send_command(self.foot.FrontTranslation(speed=-1*self.foot_base_speed, time_run=self.foot_base_time)))
        self.run_retreat_bt.grid(
            row=2, column=4, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_r_circle_bt = tk.Button(
            self.frame_3, text='R_Circle', font=button_font, width=button_w)
        self.run_r_circle_bt.bind('<Button-1>', lambda event: self._send_command(
            self.foot.TurnLeft(speed=-1*self.foot_base_speed, time_run=self.foot_base_time)))
        self.run_r_circle_bt.grid(
            row=2, column=5, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.cam_mode_bt = tk.Button(
            self.frame_3, text=self.cam_mode[0], font=button_font, width=button_w)
        self.cam_mode_bt.bind('<Button-1>', self._cam_mode_button_func)
        self.cam_mode_bt.grid(row=2, column=6, sticky=tk.NSEW,
                              padx=button_padx, pady=button_pady)

        # *******************  BUTTON ROW 3 *************
        self.sit_down_bt = tk.Button(
            self.frame_3, text='SitDown~', font=button_font, width=button_w)
        self.sit_down_bt.bind(
            '<Button-1>',
            lambda event: self._send_command(self.motion_status.StandUp(delta_x=-1 * delta_move, time_t=2000)))
        self.sit_down_bt.bind(
            '<Button-3>',
            lambda event: self._send_command(self.motion_status.StandUp(delta_x=-1 * delta_mini_move, time_t=2000)))
        self.sit_down_bt.grid(row=3, column=0, sticky=tk.NSEW,
                              padx=button_padx, pady=button_pady)

        self.load_json_bt = tk.Button(
            self.frame_3, text='LoadJson', font=button_font, width=button_w)
        self.load_json_bt.bind('<Button-1>', self._buttun_none)
        self.load_json_bt.grid(
            row=3, column=1, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.save_json_bt = tk.Button(
            self.frame_3, text='SaveJson', font=button_font, width=button_w)
        self.save_json_bt.bind('<Button-1>', self._buttun_none)
        self.save_json_bt.grid(
            row=3, column=2, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.read_mode_bt = tk.Button(
            self.frame_3, text='ReadMode', font=button_font, width=button_w)
        self.read_mode_bt.bind(
            '<Button-1>', lambda event: self._send_command(base_script.steering_engine_mode))
        self.read_mode_bt.grid(
            row=3, column=3, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.read_position_bt = tk.Button(
            self.frame_3, text='ReadPos', font=button_font, width=button_w)
        self.read_position_bt.bind(
            '<Button-1>', lambda event: self._send_command(base_script.steering_read_pos))
        self.read_position_bt.grid(
            row=3, column=4, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.hand_grap_bt = tk.Button(
            self.frame_3, text='Hand+', font=button_font, width=button_w)
        self.hand_grap_bt.bind('<Button-1>', self._test_button_func)
        self.hand_grap_bt.grid(
            row=3, column=5, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.hand_release_bt = tk.Button(
            self.frame_3, text='Hand-', font=button_font, width=button_w)
        self.hand_release_bt.bind('<Button-1>', self._buttun_none)
        self.hand_release_bt.grid(
            row=3, column=6, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        # ******************** BUTTON ROW 4************
        self.sit_down_bt = tk.Button(
            self.frame_3, text='CalibSnap', font=button_font, width=button_w)
        self.sit_down_bt.bind('<Button-1>', self._calib_capture)
        self.sit_down_bt.grid(row=4, column=0, sticky=tk.NSEW,
                              padx=button_padx, pady=button_pady)

        self.load_json_bt = tk.Button(
            self.frame_3, text='Calib', font=button_font, width=button_w)
        self.load_json_bt.bind('<Button-1>', self._cam_calib_compute)
        self.load_json_bt.grid(
            row=4, column=1, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.save_json_bt = tk.Button(
            self.frame_3, text='NULL', font=button_font, width=button_w)
        self.save_json_bt.bind('<Button-1>', self._buttun_none)
        self.save_json_bt.grid(
            row=4, column=2, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.read_mode_bt = tk.Button(
            self.frame_3, text='NULL', font=button_font, width=button_w)
        self.read_mode_bt.bind('<Button-1>', self._buttun_none)
        self.read_mode_bt.grid(
            row=4, column=3, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.read_position_bt = tk.Button(
            self.frame_3, text='NULL', font=button_font, width=button_w)
        self.read_position_bt.bind('<Button-1>', self._buttun_none)
        self.read_position_bt.grid(
            row=4, column=4, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.hand_grap_bt = tk.Button(
            self.frame_3, text='NULL', font=button_font, width=button_w)
        self.hand_grap_bt.bind('<Button-1>', self._buttun_none)
        self.hand_grap_bt.grid(
            row=4, column=5, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.hand_release_bt = tk.Button(
            self.frame_3, text='robot_init', font=button_font, width=button_w)
        self.hand_release_bt.bind('<Button-1>', self._robot_init)
        self.hand_release_bt.grid(
            row=4, column=6, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        # ******************* SERIAL ****************
        # var:
        self.var_baud_rate = 115200
        self.var_stop_bit = 1
        self.var_check_bit = 0
        self.var_bit_wide = 8
        self.var_serial_no = 4

        def _combo_change_baud_rate(var):
            self.var_baud_rate = var

        def _combo_change_stop_bit(var):
            self.var_stop_bit = var

        def _combo_change_check_bit(var):
            self.var_check_bit = var

        def _combo_change_bit_wide(var):
            self.var_bit_wide = var

        def _combo_change_serial_no(var):
            self.var_serial_no = var

        receiver_rowspan = 6
        receiver_colspan = 5
        serial_lable_height = 2
        self.receiver = tk.Text(self.frame_5, width=60, height=20)
        self.receiver.grid(
            row=0, column=0, rowspan=receiver_rowspan, columnspan=receiver_colspan)
        self.open_serial = tk.Button(self.frame_5,
                                     text=' OpenSerial ',
                                     font=button_font,
                                     width=button_w,
                                     command=self._open_serial)
        self.send_command = tk.Button(self.frame_5,
                                      text=' SEND ',
                                      font=button_font,
                                      width=button_w,
                                      command=self._send_command)
        self.stop_bit_label = tk.Label(
            self.frame_5, text='Stop Bit:', height=serial_lable_height)
        self.check_bit_label = tk.Label(
            self.frame_5, text='check_bit:', height=serial_lable_height)
        self.bit_wide_label = tk.Label(
            self.frame_5, text='bit_wide:', height=serial_lable_height)
        self.serial_no_label = tk.Label(
            self.frame_5, text='serial_no:', height=serial_lable_height)
        self.baud_rate_label = tk.Label(
            self.frame_5, text='bit_rate:', height=serial_lable_height)

        port_list = list(serial.tools.list_ports.comports())
        serial_com = []
        for m in range(len(port_list)):
            port_list_1 = list(port_list[m])
            serial_com.append(port_list_1[0])
        serial_com.append("COM0")

        self.tk_var_stop_bit = tk.StringVar()
        self.combo0 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_stop_bit,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo0['values'] = ("1", "2")
        self.combo0.bind("<<ComboboxSelected>>", lambda event: _combo_change_stop_bit(
            var=self.tk_var_stop_bit.get()))
        self.combo0.current(0)

        self.tk_var_check_bit = tk.StringVar()
        self.combo1 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_check_bit,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo1['values'] = ("NONE", "ODD", "EVEN", "MARK", "SPACE")
        self.combo1.bind("<<ComboboxSelected>>", lambda event: _combo_change_check_bit(
            var=self.tk_var_check_bit.get()))
        self.combo1.current(0)

        self.tk_var_bit_wide = tk.StringVar()
        self.combo2 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_bit_wide,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo2['values'] = ("5", "6", "7", "8")
        self.combo2.bind("<<ComboboxSelected>>", lambda event: _combo_change_bit_wide(
            var=self.tk_var_bit_wide.get()))
        self.combo2.current(3)

        self.tk_var_serial_no = tk.StringVar()
        self.combo3 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_serial_no,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo3['values'] = serial_com
        self.combo3.bind("<<ComboboxSelected>>", lambda event: _combo_change_serial_no(
            var=self.tk_var_serial_no.get()))
        self.combo3.current(0)

        self.tk_var_baud_rate = tk.StringVar()
        self.combo4 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_baud_rate,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo4['values'] = ("9600", "19200", "38400", "115200")
        self.combo4.bind("<<ComboboxSelected>>", lambda event: _combo_change_baud_rate(
            var=self.tk_var_baud_rate.get()))
        self.combo4.current(3)
        self.open_serial.grid(row=0, column=receiver_colspan)
        self.send_command.grid(row=0, column=receiver_colspan + 1)
        self.stop_bit_label.grid(row=1, column=receiver_colspan)
        self.check_bit_label.grid(row=2, column=receiver_colspan)
        self.bit_wide_label.grid(row=3, column=receiver_colspan)
        self.serial_no_label.grid(row=4, column=receiver_colspan)
        self.baud_rate_label.grid(row=5, column=receiver_colspan)
        self.combo0.grid(row=1, column=receiver_colspan + 1)
        self.combo1.grid(row=2, column=receiver_colspan + 1)
        self.combo2.grid(row=3, column=receiver_colspan + 1)
        self.combo3.grid(row=4, column=receiver_colspan + 1)
        self.combo4.grid(row=5, column=receiver_colspan + 1)

    # ***************** BUTTON FUNC *******************
    def _open_serial(self):
        self.var_baud_rate = int(self.combo4.get())
        self.var_stop_bit = int(self.combo0.get())
        self.var_check_bit = self.combo1.get()
        self.var_bit_wide = int(self.combo2.get())
        self.var_serial_no = self.combo3.get()
        if len(self.my_serial_prefer) > 0:
            self.var_serial_no = self.my_serial_prefer
        if self.var_check_bit == 'NONE':
            self.var_check_bit = serial.PARITY_NONE

        print('self.var_baud_rate:', self.var_baud_rate, '\nself.var_stop_bit:', self.var_stop_bit,
              '\nself.var_check_bit:', self.var_check_bit, '\nself.var_bit_wide:', self.var_bit_wide,
              '\nself.var_serial_no:', self.var_serial_no)
        self.my_serial = RoboSerial(com_port=self.var_serial_no,
                                    baud_rate=self.var_baud_rate,
                                    parity_check=self.var_check_bit,
                                    stop_bits=self.var_stop_bit,
                                    byte_size=self.var_bit_wide,
                                    timeout=0.3,
                                    receiver_callbacks=[self._receiver_callback])
        self.my_serial.Open()
        self._robot_init()

    def _send_command(self, command=None):
        if self.my_serial is None:
            return
        if self.my_serial.IsOpen():
            if command == None:
                self.command_send_buffer = self.send_command_entry.get()
                if len(self.command_send_buffer) > 0:
                    self.my_serial.Send(buffer=self.command_send_buffer)
                else:
                    print('NO COMMAND IN INPUT ENTRY')
            else:
                print('DEBUG SEND COMMAND: ', command)
                if isinstance(command, list):
                    for cmd in command:
                        self.my_serial.Send(buffer=cmd)
                        time.sleep(0.01)
                if isinstance(command, str):
                    self.command_send_buffer = command
                    self.my_serial.Send(buffer=command)
        else:
            print('SERIAL IS NOT OPEN')

    def _cap_curr_button_func(self, x):
        self.cap_curr_snap = False
        self.cap_curr_snapped = False

    def _cap_curr_button_right_func(self, x):
        self.cap_curr_snap = True

    def _cap_tar_button_func(self, x):
        self.cap_tar_snap = False
        self.cap_tar_snapped = False
        if self._is_all_cam_mode_no_thread():
            self._create_thread()

    def _cap_tar_button_right_func(self, x):
        self.cap_tar_snap = True

    def _cam_mode_button_func(self, x):
        self.cam_mode_flag = (self.cam_mode_flag + 1) % len(self.cam_mode)
        self.cam_mode_bt['text'] = self.cam_mode[self.cam_mode_flag]
        # self._destory_thread()
        self._create_thread()

    def _is_all_cam_mode_no_thread(self):
        for sub_thread in self.cam_mode_thread:
            if sub_thread is not None:
                return False
        return True

    def _create_thread(self):
        # self._destory_thread()
        if self.cam_mode_flag == self.thread_single_flag:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(
                target=self._thread_single)
            self.cam_mode_thread_flag = [
                False for _ in range(len(self.cam_mode_thread_flag))]
            self.cam_mode_thread_flag[self.cam_mode_flag] = True
            self.cam_mode_thread[self.cam_mode_flag].start()
        if self.cam_mode_flag == self.thread_stereo_flag:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(
                target=self._thread_stereo)
            self.cam_mode_thread_flag = [
                False for _ in range(len(self.cam_mode_thread_flag))]
            self.cam_mode_thread_flag[self.cam_mode_flag] = True
            self.cam_mode_thread[self.cam_mode_flag].start()
        if self.cam_mode_flag == self.thread_1_track_flag:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(
                target=self._thread_1_track)
            self.cam_mode_thread_flag = [
                False for _ in range(len(self.cam_mode_thread_flag))]
            self.cam_mode_thread_flag[self.cam_mode_flag] = True
            self.cam_mode_thread[self.cam_mode_flag].start()
        if self.cam_mode_flag == self.thread_1_l_track_flag:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(
                target=self._thread_1_l_track)
            self.cam_mode_thread_flag = [
                False for _ in range(len(self.cam_mode_thread_flag))]
            self.cam_mode_thread_flag[self.cam_mode_flag] = True
            self.cam_mode_thread[self.cam_mode_flag].start()
        if self.cam_mode_flag == self.thread_3d_track_flag:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(
                target=self._thread_3d_track)
            self.cam_mode_thread_flag = [
                False for _ in range(len(self.cam_mode_thread_flag))]
            self.cam_mode_thread_flag[self.cam_mode_flag] = True
            self.cam_mode_thread[self.cam_mode_flag].start()

    def _get_tk_img(self, cv_mat):
        frame = cv2.cvtColor(cv_mat, cv2.COLOR_BGR2RGBA)
        pil_img = Image.fromarray(frame)
        tk_img = ImageTk.PhotoImage(image=pil_img)
        return tk_img, cv_mat

    def _calib_capture(self, x):
        # for camera calib
        if self.calib_step == 0:
            self.receiver.insert(
                'end', 'CALIB MODE, INPUT camera_id may calibed in SEND\nexample: 1, 2')
            self.calib_step = 1
            return
        if self.calib_step == 1:
            camera_id = self.send_command_entry.get()
            if len(camera_id) > 0:
                camera_id = camera_id.split(',')
                self.calib_camera_id = [int(dd) for dd in camera_id]
                self.calib_camera_type = 'single' if len(
                    camera_id) == 1 else 'stereo'
                self.receiver.insert('end', '\nCALIB MODE, camera_id: ' + str(camera_id) + 'camera type:',
                                     self.calib_camera_type, '\nInput img_size:demo:1920,1080')
                self.calib_step = 2
            return
        if self.calib_step == 2:
            img_size = self.send_command_entry.get().split(',')
            img_size = [int(dd) for dd in img_size]
            self.calib_img_size = img_size
            self.receiver.insert(
                'end', '\ninput: calib_board_size: example: 7,7')
            self.calib_step = 3
            return

        if self.calib_step == 3:
            board_size = self.send_command_entry.get()
            board_size = board_size.split(',')
            if len(board_size) > 0:
                board_size = [int(dd) for dd in board_size]
                self.calib_board_size = board_size
                self.receiver.insert(
                    'end',
                    '\nCALIB MODE, board_size: ' + str(self.calib_board_size) + '\ninput calib pattern: default:1')
                self.calib_step = 4
            return
        if self.calib_step == 4:
            self.calib_pattern = int(self.send_command_entry.get())
            self.receiver.insert(
                'end', '\ncalib pattern: ' + str(self.calib_pattern) + '\ninput physica_size: demo: 14.0,14.0')
            self.calib_step = 5
            return
        if self.calib_step == 5:
            physics_size = self.send_command_entry.get()
            physics_size = physics_size.split(',')
            if len(physics_size) > 0:
                physics_size = [float(dd) for dd in physics_size]
                self.calib_physics_size = physics_size
                self.receiver.insert(
                    'end', '\nphysics_size:' + str(self.calib_physics_size) +
                    '\ninput save path and name: such as ./calib_saved p_main_ p_aux_ calib.xml')
                self.calib_step = 6
            return
        if self.calib_step == 6:
            calib_save_config = self.send_command_entry.get().split(' ')
            if len(calib_save_config) > 2:
                if len(self.calib_camera_id) == 1:
                    # single camera calib
                    self.calib_save_to = calib_save_config[0]
                    self.calib_cam_main_name = calib_save_config[1]
                    self.calib_cam_aux_name = 'NONE'
                    self.calib_save_name = calib_save_config[2]
                    if os.path.exists(self.calib_save_to):
                        shutil.rmtree(self.calib_save_to)
                    os.makedirs(self.calib_save_to)
                if len(self.calib_camera_id) == 2:
                    # single camera calib
                    self.calib_save_to = calib_save_config[0]
                    self.calib_cam_main_name = calib_save_config[1]
                    self.calib_cam_aux_name = calib_save_config[2]
                    self.calib_save_name = calib_save_config[3]
                    if os.path.exists(self.calib_save_to):
                        shutil.rmtree(self.calib_save_to)
                    os.makedirs(self.calib_save_to)
                self.receiver.insert(
                    'end', '\n----save folder ok: ' + self.calib_save_to + ' ' + self.calib_cam_main_name + ' ' +
                    self.calib_cam_aux_name + ' xml:' + self.calib_save_name)
                self.calib_step = 7
            return
        if self.calib_step == 7:
            if self.calib_cam_cap_thread == None:
                if len(self.calib_camera_id) == 1:
                    self.calib_cam_cap_thread = threading.Thread(
                        target=self._thread_single_cam_calib)
                    self.calib_cam_cap_thread.start()
                    self.calib_snap_count = 0
                if len(self.calib_camera_id) == 2:
                    self.calib_cam_cap_thread = threading.Thread(
                        target=self._thread_stereo_cam_calib)
                    self.calib_cam_cap_thread.start()
                    self.calib_snap_count = 0
                self.receiver.insert(
                    'end', '\n-----------CREAT CALIB STREAM-------')
            else:
                self.calib_step = 8
            return

    def _thread_single_cam_calib(self):
        print('CALIB STREAM CREATE')
        single_cam = SingleCam(cam_id=self.calib_camera_id[0],
                               cam_size=(
                                   self.calib_img_size[0], self.calib_img_size[1]),
                               cam_mode=self.calib_cam_open_mode)
        self.calib_imgs = [[]]
        open_cam_flag = None
        try:
            open_cam_flag = single_cam.OpenCam()
        except:
            return
        if open_cam_flag:
            while self.calib_step == 7 or self.calib_step == 8:
                snap = single_cam.SnapShoot()
                if snap is None:
                    break
                snap_cp = cv2.resize(snap.copy(), (512, 288))
                img_tk, snap_cp = self._get_tk_img(snap_cp)
                self.monitor_tar.configure(image=img_tk)
                self.monitor_tar.image = img_tk
                if self.calib_step == 8:
                    cv2.imwrite(
                        os.path.join(self.calib_save_to,
                                     self.calib_cam_main_name + str(self.calib_snap_count).zfill(4) + '.jpg'), snap)
                    self.calib_imgs[0].append(snap.copy())
                    self.calib_snap_count = self.calib_snap_count + 1
                    self.receiver.insert(
                        'end', '\nsnap a pic and snap total: ' + str(self.calib_snap_count))
                    self.calib_step = 7
            single_cam.Close()
            print('DEBUG CAMERA SNAP OVER')
        return

    def _thread_stereo_cam_calib(self):
        print('CALIB STREAM CREATE')
        stereo_cam = StereoCam(cam_id0=self.calib_camera_id[0],
                               cam_id1=self.calib_camera_id[1],
                               cam_size=(
                                   self.calib_img_size[0], self.calib_img_size[1]),
                               cam_mode=self.calib_cam_open_mode,
                               cam_fps=base_config.cam_fps,
                               bright=base_config.bright,
                               exposure=base_config.exposure)
        self.calib_imgs = [[], []]
        open_cam_flag = None
        # try:
        open_cam_flag = stereo_cam.OpenCam()
        # except:
        #    return
        if open_cam_flag:
            while self.calib_step == 7 or self.calib_step == 8:
                snap0, snap1 = stereo_cam.SnapShoot()
                if snap0 is None or snap1 is None:
                    print('DEBUG snap0 or snap1 None')
                    break

                ret0, corners0 = cv2.findChessboardCorners(image=snap0,
                                                           patternSize=(self.calib_board_size[1],
                                                                        self.calib_board_size[0]))
                ret1, corners1 = cv2.findChessboardCorners(image=snap1,
                                                           patternSize=(self.calib_board_size[1],
                                                                        self.calib_board_size[0]))
                snap0_cp = cv2.resize(snap0.copy(), (512, 288))
                snap1_cp = cv2.resize(snap1.copy(), (512, 288))
                ratio_x = self.calib_img_size[0] / snap0_cp.shape[1]
                ratio_y = self.calib_img_size[1] / snap0_cp.shape[0]
                if (ratio_x == ratio_y) and ret0 and ret1:
                    corners0 = corners0 / ratio_x
                    corners1 = corners1 / ratio_x
                    snap0_cp = cv2.drawChessboardCorners(image=snap0_cp,
                                                         patternSize=(
                                                             self.calib_board_size[0], self.calib_board_size[1]),
                                                         corners=corners0,
                                                         patternWasFound=ret0)
                    snap1_cp = cv2.drawChessboardCorners(image=snap1_cp,
                                                         patternSize=(self.calib_board_size[0],
                                                                      self.calib_board_size[1]),
                                                         corners=corners1,
                                                         patternWasFound=ret1)

                img_tk0, snap0_cp = self._get_tk_img(snap0_cp)
                img_tk1, snap1_cp = self._get_tk_img(snap1_cp)
                self.monitor_tar.configure(image=img_tk0)
                self.monitor_tar.image = img_tk0
                self.monitor_curr.configure(image=img_tk1)
                self.monitor_curr.image = img_tk1
                if self.calib_step == 8:
                    cv2.imwrite(
                        os.path.join(self.calib_save_to,
                                     self.calib_cam_main_name + str(self.calib_snap_count).zfill(4) + '.jpg'), snap0)
                    cv2.imwrite(
                        os.path.join(self.calib_save_to,
                                     self.calib_cam_aux_name + str(self.calib_snap_count).zfill(4) + '.jpg'), snap1)
                    self.calib_imgs[0].append(snap0.copy())
                    self.calib_imgs[1].append(snap1.copy())
                    self.calib_snap_count = self.calib_snap_count + 1
                    self.receiver.insert(
                        'end', '\nsnap a pic and snap total: ' + str(self.calib_snap_count))
                    self.calib_step = 7
            stereo_cam.Close()
            print('DEBUG CAMERA SNAP OVER')
        return

    def _cam_calib_compute(self, x):
        self.calib_step = 0
        print('DEBUG Calib Process Start')
        print('DEBUG Calib\n', 'len of imgs: ',
              len(self.calib_imgs), '\nboard size')
        if len(self.calib_camera_id) == 1:
            print('DEBUG CalibSingle Process Start')
            calib_tool = LovelyCalibTool(imgs=self.calib_imgs,
                                         board_size=self.calib_board_size,
                                         pattern=self.calib_pattern,
                                         physics_size=self.calib_physics_size,
                                         calib_save_to=self.calib_save_to,
                                         calib_save_name=self.calib_save_name)
            calib_tool.Run()
        elif len(self.calib_camera_id) == 2:
            # calib_tool_main = LovelyCalibTool(imgs=self.calib_imgs[0],
            #                                   board_size=self.calib_board_size,
            #                                   pattern=self.calib_pattern,
            #                                   physics_size=self.calib_physics_size,
            #                                   calib_save_to=self.calib_save_to,
            #                                   calib_save_name='calib_main_tmp.xml')
            # self.calib_main_cameraMatrix, self.calib_main_distCoeffs, self.calib_main_optimal_matrix = calib_tool_main.CalibOneCamera(
            # )
            # calib_tool_aux = LovelyCalibTool(imgs=self.calib_imgs[1],
            #                                  board_size=self.calib_board_size,
            #                                  pattern=self.calib_pattern,
            #                                  physics_size=self.calib_physics_size,
            #                                  calib_save_to=self.calib_save_to,
            #                                  calib_save_name='calib_aux_tmp.xml')
            # self.calib_aux_cameraMatrix, self.calib_aux_distCoeffs, self.calib_aux_optimal_matrix = calib_tool_aux.CalibOneCamera(
            # )
            print('DEBUG CalibStereo Process Start')
            calib_tool = CalibStereo(imgs=self.calib_imgs,
                                     board_size=self.calib_board_size,
                                     pattern=self.calib_pattern,
                                     physics_size=self.calib_physics_size,
                                     calib_save_to=self.calib_save_to,
                                     calib_save_name=self.calib_save_name)
            calib_tool.Run()

    def _thread_single(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        single_cam = SingleCam(cam_id=self.cam_mode_single_cam_id,
                               cam_fps=base_config.cam_fps,
                               bright=base_config.bright,
                               exposure=base_config.exposure)
        open_cam_flag = None
        try:
            open_cam_flag = single_cam.OpenCam(time_sleep=4)
        except:
            print('SUB THREAD CREATE OPEN CAM ERROR')
            return
        if open_cam_flag:
            while True and self.cam_mode_thread_flag[self.thread_single_flag]:
                snap = single_cam.SnapShoot()
                if snap is None:
                    break
                snap = cv2.undistort(snap, self.camera_matrix, self.distortion)
                snap_cp = cv2.resize(snap.copy(), (512, 288))
                img_tk, snap_cp = self._get_tk_img(snap_cp)
                if not self.cap_tar_snap:
                    self.monitor_tar.configure(image=img_tk)
                    self.monitor_tar.image = img_tk
                elif self.cap_tar_snap and not self.cap_tar_snapped:
                    self.monitor_tar.configure(image=img_tk)
                    self.monitor_tar.image = img_tk
                    self.np_tar = snap.copy()
                    self.cap_tar_snapped = True
                    #cv2.imwrite(r'D:\ubuntu18_win\rootfs\home\fq\lovely_robot\tools\tar.jpg', self.np_tar)
                if not self.cap_curr_snap:
                    self.monitor_curr.configure(image=img_tk)
                    self.monitor_curr.image = img_tk
                elif self.cap_curr_snap and not self.cap_curr_snapped:
                    self.monitor_curr.configure(image=img_tk)
                    self.monitor_curr.image = img_tk
                    self.np_curr = snap.copy()
                    self.cap_curr_snapped = True
                    dic = {'np_img1': self.np_tar, 'np_img2': self.np_curr,
                           'cam_matrix': self.camera_matrix}
                    fasci_config.SingleCamOnCall(**dic)
                    #cv2.imwrite(r'D:\ubuntu18_win\rootfs\home\fq\lovely_robot\tools\cur.jpg', self.np_curr)
                self.monitor_show.configure(image=img_tk)
                self.monitor_show.image = img_tk
                # print('I am a happy thread : ', self.cam_mode[self.cam_mode_flag])
        single_cam.Close()

        return None

    def _thread_stereo(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        stereo_cam = StereoCam(cam_id0=self.cam_mode_stereo_cam_id[0],
                               cam_id1=self.cam_mode_stereo_cam_id[1],
                               cam_size=self.cam_mode_stereo_cam_size,
                               cam_mode=self.cam_open_mode,
                               cam_fps=base_config.cam_fps,
                               bright=base_config.bright,
                               exposure=base_config.exposure)
        open_cam_flag = None
        try:
            # print('TRY OPEN CAM')
            open_cam_flag = stereo_cam.OpenCam(time_sleep=4)
        except:
            # print('TRY OPEN CAM FAIL')
            return
        if open_cam_flag:
            while True and self.cam_mode_thread_flag[self.thread_stereo_flag]:
                snap0, snap1 = stereo_cam.SnapShoot()
                if snap0 is None or snap1 is None:
                    break
                snap0 = cv2.resize(snap0, (512, 288))
                snap1 = cv2.resize(snap1, (512, 288))
                # snap = cv2.undistort(snap, self.camera_matrix, self.distortion)
                img_tk0, snap0 = self._get_tk_img(snap0)
                self.monitor_tar.configure(image=img_tk0)
                self.monitor_tar.image = img_tk0

                # snap = cv2.undistort(snap, self.camera_matrix, self.distortion)
                img_tk1, snap1 = self._get_tk_img(snap1)
                self.monitor_curr.configure(image=img_tk1)
                self.monitor_curr.image = img_tk1
        stereo_cam.Close()

        return None

    def _thread_1_track(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        single_cam = SingleCam(cam_id=self.cam_mode_single_cam_id,
                               cam_fps=base_config.cam_fps,
                               bright=base_config.bright,
                               exposure=base_config.exposure)
        open_cam_flag = None
        try:
            print('TRY OPEN CAM')
            open_cam_flag = single_cam.OpenCam(time_sleep=4)
        except:
            return
        if open_cam_flag:
            while True and self.cam_mode_thread_flag[self.thread_1_track_flag]:

                snap = single_cam.SnapShoot()
                if snap is None:
                    print('DEBUG snap is NULL')
                    break
                #snap = cv2.undistort(snap, self.camera_matrix, self.distortion)
                snap_cp = cv2.resize(snap.copy(), (512, 288))
                img_tk, snap_cp = self._get_tk_img(snap_cp)
                if not self.cap_tar_snap:
                    self.monitor_tar.configure(image=img_tk)
                    self.monitor_tar.image = img_tk
                elif self.cap_tar_snap and not self.cap_tar_snapped:
                    self.monitor_tar.configure(image=img_tk)
                    self.monitor_tar.image = img_tk
                    self.np_tar = snap.copy()
                    self.cap_tar_snapped = True
                else:
                    self.monitor_curr.configure(image=img_tk)
                    self.monitor_curr.image = img_tk
                    self.np_curr = snap.copy()
                    self.cap_curr_snapped = True
                    dic = {'np_img1': self.np_tar, 'np_img2': self.np_curr,
                           'cam_matrix': self.camera_matrix}
                    dic = fasci_config.SingleCamOnCall(**dic)
                    self._send_command(
                        self.motion_status.ConditionReflex(**dic, time_t=1000))
                self.monitor_show.configure(image=img_tk)
                self.monitor_show.image = img_tk
                # time.sleep(2)
        single_cam.Close()

        return

    def _thread_1_l_track(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        while True and self.cam_mode_thread_flag[self.thread_1_l_track_flag]:
            print('I am a happy thread : ', self.cam_mode[self.cam_mode_flag])

        return

    def _thread_3d_track(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        while True and self.cam_mode_thread_flag[self.thread_3d_track_flag]:
            print('I am a happy thread : ', self.cam_mode[self.cam_mode_flag])

        return

    def _destory_thread(self):
        for idx, thread_idx in enumerate(self.cam_mode_thread):
            if thread_idx is not None:
                try:
                    self._async_raise(thread_idx.ident, SystemExit)
                except:
                    print('ASYNC_RAISE\n')

                self.cam_mode_thread[idx] = None
                print('SUB THREAD KILL : ', self.cam_mode[idx])

    def _async_raise(self, tid, exctype):
        """raises the exception, performs cleanup if needed"""
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
            tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            # """if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"""
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    def _test_button_func(self, x):
        if x != None:
            print('x')
        for tt in range(5):
            self._send_command(base_script.steering_straighten)
            print('send_command1', ' ', tt)
            time.sleep(5)
            self._send_command(base_script.steering_half_straighten)
            print('send_command2', ' ', tt)
            time.sleep(5)

    def _buttun_none(self, x):
        print('BUTTON_DOWN_\n', x)
        return

    def _receiver_callback(self, buffer_in):
        print('DEBUG RECEIVER_CALL_BACK: ', time.time())
        self.receiver.insert('end', buffer_in)

    def _robot_init(self, x=None):
        serial_callback = [self.robot_arm.UpdateSteerPositionBySerialReturn]
        self.my_serial.SetReceiverCallBack(serial_callback)
        time.sleep(1)
        self._send_command(base_script.steering_read_pos)
        time.sleep(1)
        self.motion_status.UpdateCurrPwmFromSerial(
            self.my_serial.receiver_buffer)
        self.my_serial.SetReceiverCallBack([self._receiver_callback])

    def Run(self):
        self.window.mainloop()


if __name__ == '__main__':
    window = TuningGUI()
    window.Run()
