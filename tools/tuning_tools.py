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


class SerialRobot:
    def __init__(
        self,
        com_port='COM0',
        baud_rate=115200,
        parity_check=serial.PARITY_NONE,
        stop_bits=1,
        byte_size=serial.EIGHTBITS,
        timeout=0.2,
        receiver_callbacks=[],
    ):
        self.port = com_port
        self.baud_rate = baud_rate
        self.parity_check = parity_check
        self.stop_bits = stop_bits
        self.byte_size = byte_size
        self.timeout = timeout
        self.receiver_thread = None
        self.receiver_buffer = ''
        self.my_serial = None
        self.receiver_callbacks = receiver_callbacks

    def SetReceiverCallBack(self, call_backs):
        self.receiver_callbacks = call_backs
        return

    def IsOpen(self):
        if self.my_serial == None:
            return False
        else:
            return self.my_serial.isOpen()

    def _thread_receive(self):
        while True:
            read = self.my_serial.readall()
            if len(read) > 0:
                self.receiver_buffer = str(bytes(read).decode('utf-8', "ignore"))
                for func in self.receiver_callbacks:
                    func(self.receiver_buffer)
                print(self.receiver_buffer, ' ', self.receiver_callbacks)

    def Open(self):
        self.my_serial = serial.Serial(port=self.port,
                                       baudrate=self.baud_rate,
                                       parity=self.parity_check,
                                       timeout=self.timeout,
                                       stopbits=self.stop_bits,
                                       bytesize=self.byte_size)
        self.receiver_thread = threading.Thread(target=self._thread_receive)
        self.receiver_thread.start()
        # self.receiver_callback('Open OK')

    def Send(self, buffer):
        status_ = self.my_serial.isOpen()
        if status_ == True:
            self.my_serial.write(buffer.encode('ascii'))
        else:
            print('ERROR IN SERIAL SEND! SERIAL IS NOT OPEN\n')

    def _async_raise(self, tid, exctype):
        """raises the exception, performs cleanup if needed"""
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            # """if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"""
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    def Close(self):
        self.my_serial.close()
        self._async_raise(self.receiver_thread.ident, SystemExit)


class TuningGUI:
    def __init__(self):
        tk_tittle = 'tuning_tools'
        self.window = tk.Tk()
        self.window.title(tk_tittle)
        self.frame_main = self.window

        # *************************** app run var *********************
        self.my_serial = None
        self.command_send_buffer = ''
        # single camera \ stereo_camera \ single_track \ stereo_left_track \ stereo_3d_track
        self.cam_mode = ['SingleCam', 'Stereo', '1_Track', '1_L_Track', '3DTrack']
        self.cam_mode_flag = 0
        self.cam_mode_thread = [None] * len(self.cam_mode)
        
        # click for grap video stream right click for snap
        self.cap_curr_snap = False
        self.cap_tar_snap = False
        self.cap_curr_snapped = False
        self.cap_tar_snapped = False
        self.cam_mode_single_cam_id = base_script.cam_mode_single_cam_id
        self.cam_mode_stereo_cam_id = base_script.cam_mode_stereo_cam_id
        self.cam_mode_stereo_cam_size = base_script.cam_mode_stereo_cam_size
        self.cam_open_mode = base_script.cam_open_mode

        # **** Read Calib single camera
        fs_read_calib = cv2.FileStorage(base_config.calib_path, cv2.FileStorage_READ)
        self.camera_matrix = fs_read_calib.getNode('camera_matrix').mat()
        self.optimal_matrix = fs_read_calib.getNode('optimal_matrix').mat()
        self.distortion = fs_read_calib.getNode('distortion').mat()
        fs_read_calib.release()

        # define robot arm
        self.robot_arm = RobotArm()

        # **************************************************************
        self.monitor = tk.PanedWindow(self.frame_main, orient=tk.VERTICAL)
        self.euler_show = tk.PanedWindow(self.frame_main, orient=tk.VERTICAL)

        self.type_in = tk.PanedWindow(self.frame_main, orient=tk.VERTICAL)

        self.buttons_and_serial = tk.Frame(self.frame_main)
        self.buttons = tk.PanedWindow(self.buttons_and_serial, orient=tk.VERTICAL)
        self.serial_com = tk.PanedWindow(self.buttons_and_serial, orient=tk.VERTICAL)

        self.frame_1 = tk.LabelFrame(master=self.monitor, bg='#555555555', text='monitor')
        self.frame_2 = tk.LabelFrame(master=self.euler_show, bg='#555555555', text='euler_show')
        self.frame_3 = tk.LabelFrame(master=self.buttons, bg='#555555555', text='control')
        self.frame_4 = tk.LabelFrame(master=self.type_in, bg='#555555555', text='input')
        self.frame_5 = tk.LabelFrame(master=self.serial_com, bg='#555555555', text='serial')

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
        self.monitor_curr = tk.Canvas(self.frame_1, bg='#345645323', width=self.pic_width, height=self.pic_height)
        self.monitor_curr = tk.Label(master=self.frame_1,
                                     image=self.monitor_curr_img,
                                     height=self.pic_height,
                                     width=self.pic_width)
        self.monitor_curr.pack(side=tk.LEFT)

        self.np_show = empty_img.copy().astype(np.uint8)
        self.image_show = Image.fromarray(self.np_show)
        self.monitor_show_img = ImageTk.PhotoImage(self.image_show)
        self.monitor_show = tk.Canvas(self.frame_1, bg='#345645323', width=self.pic_width, height=self.pic_height)
        self.monitor_show = tk.Label(master=self.frame_1,
                                     image=self.monitor_show_img,
                                     height=self.pic_height,
                                     width=self.pic_width)
        self.monitor_show.pack(side=tk.LEFT)

        # ********************** EULER_SHOW *********************
        font_height = 2
        font_width = 34
        euler_show_padx = 4
        self.pitch = tk.Label(self.frame_2, text='Pitch_x', height=font_height, width=font_width, bg='#000fff000')
        self.pitch.grid(row=0, column=0, sticky=tk.NSEW, padx=euler_show_padx, pady=1, ipadx=1, ipady=1)
        self.yaw = tk.Label(self.frame_2, text='Yaw_y', height=font_height, width=font_width, bg='#000fff000')
        self.yaw.grid(row=0, column=1, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.roll = tk.Label(self.frame_2, text='Roll_z', height=font_height, width=font_width, bg='#000fff000')
        self.roll.grid(row=0, column=2, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.shift_x = tk.Label(self.frame_2, text='Shift_x', height=font_height, width=font_width, bg='#000fff000')
        self.shift_x.grid(row=0, column=3, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.shift_y = tk.Label(self.frame_2, text='Shift_y', height=font_height, width=font_width, bg='#000fff000')
        self.shift_y.grid(row=0, column=4, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.shift_z = tk.Label(self.frame_2, text='Shift_z', height=font_height, width=font_width, bg='#000fff000')
        self.shift_z.grid(row=0, column=5, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        # ------
        font_height_val = 2
        self.pitch_val = tk.Label(self.frame_2, text='0.00', height=font_height_val)
        self.pitch_val.grid(row=1, column=0, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.yaw_val = tk.Label(self.frame_2, text='0.00', height=font_height_val)
        self.yaw_val.grid(row=1, column=1, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.roll_val = tk.Label(self.frame_2, text='0.00', height=font_height_val)
        self.roll_val.grid(row=1, column=2, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.shift_x_val = tk.Label(self.frame_2, text='0.00', height=font_height_val)
        self.shift_x_val.grid(row=1, column=3, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.shift_y_val = tk.Label(self.frame_2, text='0.00', height=font_height_val)
        self.shift_y_val.grid(row=1, column=4, sticky=tk.NSEW, padx=euler_show_padx, pady=1)
        self.shift_z_val = tk.Label(self.frame_2, text='0.00', height=font_height_val)
        self.shift_z_val.grid(row=1, column=5, sticky=tk.NSEW, padx=euler_show_padx, pady=1)

        # ******************* TYPE IN *********************
        self.json_label = tk.Label(self.frame_4, text='Json:', height=3)
        path_default = tk.StringVar(value='/home/fq/lovely_robot/script/init_script.json')
        self.json_path = tk.Entry(self.frame_4, font=('Arial', 16), width=55, textvariable=path_default)
        self.send_label = tk.Label(self.frame_4, text='Send:', height=3)
        self.send_command_entry = tk.Entry(self.frame_4, font=('Arial', 16), width=65)
        self.json_label.pack(side=tk.LEFT, padx=3, pady=3)
        self.json_path.pack(side=tk.LEFT)
        self.send_label.pack(side=tk.LEFT, padx=3, pady=3)
        self.send_command_entry.pack(side=tk.LEFT)

        # *********************** BUTTON *****************
        button_font = ('Arial', 12)
        button_w = 9
        button_padx = 7
        button_pady = 7
        delta_control = 4
        self.pitch_p_bt = tk.Button(self.frame_3, text='Pitch+^', font=button_font, width=button_w)
        self.pitch_p_bt.bind('<Button-1>',
                             lambda event: self._send_command(self.robot_arm.PitchUp(delta_x=delta_control, time_t=2000)))
        self.pitch_p_bt.grid(row=0, column=0, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.yaw_p_bt = tk.Button(self.frame_3, text='Yaw+>', font=button_font, width=button_w)
        self.yaw_p_bt.bind('<Button-1>',
                           lambda event: self._send_command(self.robot_arm.YawUp(delta_x=delta_control, time_t=2000)))
        self.yaw_p_bt.grid(row=0, column=1, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.roll_p_bt = tk.Button(self.frame_3, text='Roll+@', font=button_font, width=button_w)
        self.roll_p_bt.bind('<Button-1>',
                            lambda event: self._send_command(self.robot_arm.RollUp(delta_x=delta_control, time_t=2000)))
        self.roll_p_bt.grid(row=0, column=2, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_lf_bt = tk.Button(self.frame_3, text='LF', font=button_font, width=button_w)
        self.run_lf_bt.bind('<Button-1>', self._buttun_none)
        self.run_lf_bt.grid(row=0, column=3, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_f_bt = tk.Button(self.frame_3, text='Forward ^', font=button_font, width=button_w)
        self.run_f_bt.bind('<Button-1>', self._buttun_none)
        self.run_f_bt.grid(row=0, column=4, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_rf_bt = tk.Button(self.frame_3, text='RF', font=button_font, width=button_w)
        self.run_rf_bt.bind('<Button-1>', self._buttun_none)
        self.run_rf_bt.grid(row=0, column=5, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.cap_tar_bt = tk.Button(self.frame_3, text='CapTar', font=button_font, width=button_w)
        self.cap_tar_bt.bind('<Button-1>', self._cap_tar_button_func)
        self.cap_tar_bt.bind('<Button-3>', self._cap_tar_button_right_func)
        self.cap_tar_bt.grid(row=0, column=6, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.pitch_m_bt = tk.Button(self.frame_3, text='Pitch-v', font=button_font, width=button_w)
        self.pitch_m_bt.bind('<Button-1>',
                             lambda event: self._send_command(self.robot_arm.PitchUp(delta_x=-1*delta_control, time_t=2000)))
        self.pitch_m_bt.grid(row=1, column=0, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.yaw_m_bt = tk.Button(self.frame_3, text='Yaw-<', font=button_font, width=button_w)
        self.yaw_m_bt.bind('<Button-1>',lambda event: self._send_command(self.robot_arm.YawUp(delta_x=-1*delta_control, time_t=2000)))
        self.yaw_m_bt.grid(row=1, column=1, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.roll_m_bt = tk.Button(self.frame_3, text='Roll-G', font=button_font, width=button_w)
        self.roll_m_bt.bind('<Button-1>', lambda event: self._send_command(self.robot_arm.RollUp(delta_x=-1*delta_control, time_t=2000)))
        self.roll_m_bt.grid(row=1, column=2, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_left_bt = tk.Button(self.frame_3, text='Left <', font=button_font, width=button_w)
        self.run_left_bt.bind('<Button-1>', self._buttun_none)
        self.run_left_bt.grid(row=1, column=3, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_pause_bt = tk.Button(self.frame_3, text='Run', font=button_font, width=button_w)
        self.run_pause_bt.bind('<Button-1>', self._buttun_none)
        self.run_pause_bt.grid(row=1, column=4, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_right_bt = tk.Button(self.frame_3, text='Right >', font=button_font, width=button_w)
        self.run_right_bt.bind('<Button-1>', self._buttun_none)
        self.run_right_bt.grid(row=1, column=5, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.cap_curr_bt = tk.Button(self.frame_3, text='CapCurr', font=button_font, width=button_w)
        self.cap_curr_bt.bind('<Button-1>', self._cap_curr_button_func)
        self.cap_curr_bt.bind('<Button-3>', self._cap_curr_button_right_func)
        self.cap_curr_bt.grid(row=1, column=6, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.stand_up_bt = tk.Button(self.frame_3, text='StandUp~', font=button_font, width=button_w)
        self.stand_up_bt.bind('<Button-1>',lambda event: self._send_command(self.robot_arm.StandUp(delta_x=delta_control, time_t=2000)))
        self.stand_up_bt.grid(row=2, column=0, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.release_machine_bt = tk.Button(self.frame_3, text='Release', font=button_font, width=button_w)
        self.release_machine_bt.bind('<Button-1>', lambda event: self._send_command(base_script.steering_release_mode))
        self.release_machine_bt.grid(row=2, column=1, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.regain_bt = tk.Button(self.frame_3, text='Regain', font=button_font, width=button_w)
        self.regain_bt.bind('<Button-1>', lambda event: self._send_command(base_script.steering_regain_mode))
        self.regain_bt.grid(row=2, column=2, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_l_circle_bt = tk.Button(self.frame_3, text='L_Circle', font=button_font, width=button_w)
        self.run_l_circle_bt.bind('<Button-1>', self._buttun_none)
        self.run_l_circle_bt.grid(row=2, column=3, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_retreat_bt = tk.Button(self.frame_3, text='Retreat v', font=button_font, width=button_w)
        self.run_retreat_bt.bind('<Button-1>', self._buttun_none)
        self.run_retreat_bt.grid(row=2, column=4, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.run_r_circle_bt = tk.Button(self.frame_3, text='R_Circle', font=button_font, width=button_w)
        self.run_r_circle_bt.bind('<Button-1>', self._buttun_none)
        self.run_r_circle_bt.grid(row=2, column=5, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.cam_mode_bt = tk.Button(self.frame_3, text=self.cam_mode[0], font=button_font, width=button_w)
        self.cam_mode_bt.bind('<Button-1>', self._cam_mode_button_func)
        self.cam_mode_bt.grid(row=2, column=6, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.sit_down_bt = tk.Button(self.frame_3, text='SitDown~', font=button_font, width=button_w)
        self.sit_down_bt.bind('<Button-1>', lambda event: self._send_command(self.robot_arm.StandUp(delta_x=-1*delta_control, time_t=2000)))
        self.sit_down_bt.grid(row=3, column=0, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.load_json_bt = tk.Button(self.frame_3, text='LoadJson', font=button_font, width=button_w)
        self.load_json_bt.bind('<Button-1>', self._buttun_none)
        self.load_json_bt.grid(row=3, column=1, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.save_json_bt = tk.Button(self.frame_3, text='SaveJson', font=button_font, width=button_w)
        self.save_json_bt.bind('<Button-1>', self._buttun_none)
        self.save_json_bt.grid(row=3, column=2, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.read_mode_bt = tk.Button(self.frame_3, text='ReadMode', font=button_font, width=button_w)
        self.read_mode_bt.bind('<Button-1>', lambda event: self._send_command(base_script.steering_engine_mode))
        self.read_mode_bt.grid(row=3, column=3, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.read_position_bt = tk.Button(self.frame_3, text='ReadPos', font=button_font, width=button_w)
        self.read_position_bt.bind('<Button-1>', lambda event: self._send_command(base_script.steering_read_pos))
        self.read_position_bt.grid(row=3, column=4, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.hand_grap_bt = tk.Button(self.frame_3, text='Hand+', font=button_font, width=button_w)
        self.hand_grap_bt.bind('<Button-1>', self._test_button_func)
        self.hand_grap_bt.grid(row=3, column=5, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

        self.hand_release_bt = tk.Button(self.frame_3, text='Hand-', font=button_font, width=button_w)
        self.hand_release_bt.bind('<Button-1>', self._buttun_none)
        self.hand_release_bt.grid(row=3, column=6, sticky=tk.NSEW, padx=button_padx, pady=button_pady)

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
        self.receiver.grid(row=0, column=0, rowspan=receiver_rowspan, columnspan=receiver_colspan)
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
        self.stop_bit_label = tk.Label(self.frame_5, text='Stop Bit:', height=serial_lable_height)
        self.check_bit_label = tk.Label(self.frame_5, text='check_bit:', height=serial_lable_height)
        self.bit_wide_label = tk.Label(self.frame_5, text='bit_wide:', height=serial_lable_height)
        self.serial_no_label = tk.Label(self.frame_5, text='serial_no:', height=serial_lable_height)
        self.baud_rate_label = tk.Label(self.frame_5, text='bit_rate:', height=serial_lable_height)

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
        self.combo0.bind("<<ComboboxSelected>>", lambda event: _combo_change_stop_bit(var=self.tk_var_stop_bit.get()))
        self.combo0.current(0)

        self.tk_var_check_bit = tk.StringVar()
        self.combo1 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_check_bit,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo1['values'] = ("NONE", "ODD", "EVEN", "MARK", "SPACE")
        self.combo1.bind("<<ComboboxSelected>>", lambda event: _combo_change_check_bit(var=self.tk_var_check_bit.get()))
        self.combo1.current(0)

        self.tk_var_bit_wide = tk.StringVar()
        self.combo2 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_bit_wide,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo2['values'] = ("5", "6", "7", "8")
        self.combo2.bind("<<ComboboxSelected>>", lambda event: _combo_change_bit_wide(var=self.tk_var_bit_wide.get()))
        self.combo2.current(3)

        self.tk_var_serial_no = tk.StringVar()
        self.combo3 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_serial_no,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo3['values'] = serial_com
        self.combo3.bind("<<ComboboxSelected>>", lambda event: _combo_change_serial_no(var=self.tk_var_serial_no.get()))
        self.combo3.current(0)

        self.tk_var_baud_rate = tk.StringVar()
        self.combo4 = ttk.Combobox(self.frame_5,
                                   textvariable=self.tk_var_baud_rate,
                                   width=8,
                                   height=2,
                                   justify=tk.CENTER)
        self.combo4['values'] = ("9600", "19200", "38400", "115200")
        self.combo4.bind("<<ComboboxSelected>>", lambda event: _combo_change_baud_rate(var=self.tk_var_baud_rate.get()))
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
        if self.var_check_bit == 'NONE':
            self.var_check_bit = serial.PARITY_NONE

        print('self.var_baud_rate:', self.var_baud_rate, '\nself.var_stop_bit:', self.var_stop_bit,
              '\nself.var_check_bit:', self.var_check_bit, '\nself.var_bit_wide:', self.var_bit_wide,
              '\nself.var_serial_no:', self.var_serial_no)
        self.my_serial = SerialRobot(com_port=self.var_serial_no,
                                     baud_rate=self.var_baud_rate,
                                     parity_check=self.var_check_bit,
                                     stop_bits=self.var_stop_bit,
                                     byte_size=self.var_bit_wide,
                                     timeout=0.3,
                                     receiver_callbacks=[self._receiver_callback])
        self.my_serial.Open()
        self._robot_init()

    def _send_command(self, command=None):
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
        
        if self.cam_mode_flag == 0:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(target=self._thread_single)
            self.cam_mode_thread[self.cam_mode_flag].start()
        if self.cam_mode_flag == 1:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(target=self._thread_stereo)
            self.cam_mode_thread[self.cam_mode_flag].start()
        if self.cam_mode_flag == 2:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(target=self._thread_1_track)
            self.cam_mode_thread[self.cam_mode_flag].start()
        if self.cam_mode_flag == 3:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(target=self._thread_1_l_track)
            self.cam_mode_thread[self.cam_mode_flag].start()
        if self.cam_mode_flag == 4:
            self.cam_mode_thread[self.cam_mode_flag] = threading.Thread(target=self._thread_3d_track)
            self.cam_mode_thread[self.cam_mode_flag].start()

    def _get_tk_img(self, cv_mat):
        frame = cv2.cvtColor(cv_mat, cv2.COLOR_BGR2RGBA)
        pil_img = Image.fromarray(frame)
        tk_img = ImageTk.PhotoImage(image=pil_img)
        return tk_img, cv_mat

    def _thread_single(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        sigle_cam = SingleCam(cam_id=self.cam_mode_single_cam_id)
        if sigle_cam.OpenCam():
            while True:
                snap = sigle_cam.SnapShoot()
                if snap is None:
                    break
                snap = cv2.undistort(snap, self.camera_matrix, self.distortion)
                img_tk, snap = self._get_tk_img(snap)
                if not self.cap_tar_snap:
                    self.monitor_tar.configure(image=img_tk)
                    self.monitor_tar.image = img_tk
                elif self.cap_tar_snap and not self.cap_tar_snapped:
                    self.monitor_tar.configure(image=img_tk)
                    self.monitor_tar.image = img_tk
                    self.np_tar = snap.copy()
                    self.cap_tar_snapped = True
                if not self.cap_curr_snap:
                    self.monitor_curr.configure(image=img_tk)
                    self.monitor_curr.image = img_tk
                elif self.cap_curr_snap and not self.cap_curr_snapped:
                    self.monitor_curr.configure(image=img_tk)
                    self.monitor_curr.image = img_tk
                    self.np_curr = snap.copy()
                    self.cap_curr_snapped = True
                    dic = {'np_img1': self.np_tar, 'np_img2': self.np_curr, 'cam_matrix': self.camera_matrix}
                    fasci_config.SingleCamOnCall(**dic)
                self.monitor_show.configure(image=img_tk)
                self.monitor_show.image = img_tk
                # print('I am a happy thread : ', self.cam_mode[self.cam_mode_flag])
        sigle_cam.Close()
        
        return None

    def _thread_stereo(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        stereo_cam = StereoCam(cam_id0=self.cam_mode_stereo_cam_id[0],
                               cam_id1=self.cam_mode_stereo_cam_id[1],
                               cam_size=self.cam_mode_stereo_cam_size,
                               cam_mode=self.cam_open_mode)
        if stereo_cam.OpenCam():
            while True:
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
        sigle_cam = SingleCam(cam_id=self.cam_mode_single_cam_id)
        if sigle_cam.OpenCam():
            while True:
                snap = sigle_cam.SnapShoot()
                if snap is None:
                    break
                snap = cv2.undistort(snap, self.camera_matrix, self.distortion)
                img_tk, snap = self._get_tk_img(snap)
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
                    dic = {'np_img1': self.np_tar, 'np_img2': self.np_curr, 'cam_matrix': self.camera_matrix}
                    fasci_config.SingleCamOnCall(**dic)
                self.monitor_show.configure(image=img_tk)
                self.monitor_show.image = img_tk
                time.sleep(2)
        sigle_cam.Close()
        
        return

    def _thread_1_l_track(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        while True:
            print('I am a happy thread : ', self.cam_mode[self.cam_mode_flag])
        
        return

    def _thread_3d_track(self):
        print('SUB THREAD CREATE : ', self.cam_mode[self.cam_mode_flag])
        while True:
            print('I am a happy thread : ', self.cam_mode[self.cam_mode_flag])
        
        return

    def _destory_thread(self):
        for idx, thread_idx in enumerate(self.cam_mode_thread):
            if thread_idx is not None:
                self._async_raise(thread_idx.ident, SystemExit)
                self.cam_mode_thread[idx] = None
                print('SUB THREAD KILL : ', self.cam_mode[idx])

    def _async_raise(self, tid, exctype):
        """raises the exception, performs cleanup if needed"""
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
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

    def _robot_init(self):
        serial_callback = [self.robot_arm.UpdateSteerPositionBySerialReturn]
        self.my_serial.SetReceiverCallBack(serial_callback)
        time.sleep(1)
        self._send_command(base_script.steering_read_pos)
        time.sleep(1)
        self.robot_arm.UpdateSteerPositionBySerialReturn(self.my_serial.receiver_buffer)
        self.my_serial.SetReceiverCallBack([self._receiver_callback])

    def Run(self):
        self.window.mainloop()


if __name__ == '__main__':
    window = TuningGUI()
    window.Run()
