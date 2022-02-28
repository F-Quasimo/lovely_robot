from email.policy import default
import os
import tkinter as tk
import cv2
from PIL import Image, ImageTk
import numpy as np
import shutil

# cap = cv2.VideoCapture(1)
# while 1:
#     ret, frame = cap.read()
#     cv2.imshow("cap", frame)
#     if cv2.waitKey(100) & 0xff == ord('q'):
#         break
# cap.release()
# cv2.destroyAllWindows()


class LovelyCalibTool:
    def __init__(self,
                 camera_type='single',
                 imgs=[[]],
                 board_size=(7, 7),
                 pattern=1,
                 physics_size=(16.4375, 15.875),
                 calib_save_to='.'):
        # board_size=(width,height)
        # board_size: inner corners, w - h, pattern 1: chess; 0: circle; physics_size:w-h
        self.camera_type = camera_type
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
        self.calib_save_to = os.path.join(self.save_to, 'calib.xml')

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
            save_2 = os.path.join(self.save_to, str(idx) + '.jpg')
            img_u = cv2.undistort(img, self.matrix, self.dist, self.optimal_matrix)
            cv2.imwrite(save_2, img_u)

    def Run(self):
        if self.camera_type == 'single':
            imgs_corners = []
            objs_corners = []
            if self.pattern == 1:
                self._calib_one_camera()
                self._save_calib()
                self._debug_test()
        return


class TKWindow():
    def __init__(self):
        tk_title = 'camera calibration tools'
        window_size = '1200x800'
        self.window = tk.Tk()
        self.window.title(tk_title)
        #self.window.geometry(window_size)

        self.frame_main = self.window
        self.frame_1 = tk.Frame(master=self.frame_main, bg='#000fff000', borderwidth=10)
        self.frame_2 = tk.Frame(master=self.frame_main, bg='#ffffff000', borderwidth=10)
        self.frame_3 = tk.Frame(master=self.frame_main, bg='#000ffffff', borderwidth=10)
        self.frame_4 = tk.Frame(master=self.frame_main, bg='#000000fff', borderwidth=10)

        # **************** GUI ROW 0 *************************
        self.camera_select_lable = tk.Label(self.frame_1, text='Camera id:(0,1,2)', font=('Arial', 12), borderwidth=2)
        self.camera_select_lable.grid(row=0, sticky=tk.W)
        self.input_camera_selected = tk.Entry(self.frame_1, font=('Arial', 12), width=10)
        self.input_camera_selected.grid(row=0, column=1, sticky=tk.E)
        self.camera_select_check_bt = tk.Button(self.frame_1,
                                                text='This_camera_check',
                                                font=('Arial', 12),
                                                command=self.GetCamera_ID,
                                                borderwidth=2)
        self.camera_select_check_bt.grid(row=0, column=2, sticky=tk.E)

        self.chess_info_lable = tk.Label(self.frame_1,
                                         text='chess_info:(0:inner_w,1:inner_h,2:pt,3:frames_n,physics_size_4:X,5:Y)',
                                         font=('Arial', 12))
        self.chess_info_lable.grid(row=1, column=0, sticky=tk.W)
        self.input_chess_info = tk.Entry(self.frame_1, font=('Arial', 12), width=15)
        self.input_chess_info.grid(row=1, column=1, sticky=tk.E)
        self.chess_info_check_bt = tk.Button(self.frame_1,
                                             text='chess_info_check',
                                             font=('Arial', 12),
                                             command=self.GetCamera_ID,
                                             borderwidth=2)
        self.chess_info_check_bt.grid(row=1, column=2, sticky=tk.E)

        self.frame_1.pack(side='top', fill='both', expand='NO')
        # **************** GUI ROW 1*********************
        self.monitor_width = 1100
        self.monitor_height = 800
        self.monitor = tk.Canvas(self.frame_2, bg='#345645323', width=self.monitor_width, height=self.monitor_height)
        self.monitor.pack()
        self.monitor_know_its_size = False

        self.path_to_lable = tk.Label(self.frame_3, text='save_to:', bg='#446753444', font=('Arial', 12))
        self.path_to_lable.grid(row=0, sticky=tk.W)

        path_default = tk.StringVar(value='./calib_saved')
        self.input_path_to = tk.Entry(self.frame_3, font=('Arial', 12), textvariable=path_default)
        self.input_path_to.grid(row=0, column=1)
        self.path_ok_bt = tk.Button(self.frame_3,
                                    text='Check_path_open_camera',
                                    font=('Arial', 12),
                                    command=self.Check_Path,
                                    borderwidth=2)
        self.path_ok_bt.grid(row=0, column=2)
        self.frame_3.pack(side='top', fill='both', expand='NO')

        self.capture_bt = tk.Button(self.frame_4,
                                    text='CAPTURE',
                                    font=('Arial', 12),
                                    command=self.CAPTURE_BT,
                                    borderwidth=2)
        self.capture_flag = False
        self.capture_count = 0
        self.capture_bt.pack(side='left')
        self.calib_bt = tk.Button(self.frame_4, text='CALIB', font=('Arial', 12), command=self.CALIB, borderwidth=2)
        self.calib_bt.pack(side='left')
        self.frame_4.pack()

        self.camera_used = []
        self.chess_info = []
        self.saved_path_to = ''
        self.imgs = []

    def CAPTURE_BT(self):
        self.capture_flag = True

    def CALIB(self):
        self.saved_path_to = self.input_path_to.get()
        print('DEBUG 2: ', self.saved_path_to)
        if os.path.exists(self.saved_path_to):
            self.imgs = []
            files = os.listdir(self.saved_path_to)
            files.sort()
            main_files = [dd for dd in files if 'p_main' in dd]
            aux_files = [dd for dd in files if 'p_aux' in dd]
            # aa = cv2.imread(os.path.join(self.saved_path_to,main_files[0]))
            main_files = [cv2.imread(os.path.join(self.saved_path_to, dd)) for dd in main_files]
            aux_files = [cv2.imread(os.path.join(self.saved_path_to, dd)) for dd in aux_files]
            if len(main_files) > 0:
                self.imgs.append(main_files)
            if len(aux_files) > 0:
                self.imgs.append(aux_files)
        print('DEBUF SAVED: ', len(self.imgs), ' ', len(self.imgs[0]))

        calib = LovelyCalibTool(camera_type='single',
                                imgs=self.imgs,
                                board_size=(7, 7),
                                pattern=1,
                                physics_size=(16.4375, 15.875),
                                calib_save_to=self.saved_path_to)
        calib.Run()
        return

    def OpenCamera(self):
        def _get_tk_img(cap):
            ref, frame_cv2 = cap.read()
            frame = cv2.cvtColor(frame_cv2, cv2.COLOR_BGR2RGBA)
            pil_img = Image.fromarray(frame)
            tk_img = ImageTk.PhotoImage(image=pil_img)
            return tk_img, frame_cv2

        if len(self.camera_used) == 1:
            cap = cv2.VideoCapture(self.camera_used[0])
            while True:
                img_tk, frame_cv2 = _get_tk_img(cap)
                if self.capture_flag == True:
                    save_name = os.path.join(self.saved_path_to, 'p_main_' + str(self.capture_count) + '.jpg')
                    self.capture_count = self.capture_count + 1
                    cv2.imwrite(save_name, frame_cv2)
                    self.capture_flag = False
                self.monitor.create_image(0, 0, anchor=tk.NW, image=img_tk)
                if not self.monitor_know_its_size:
                    self.monitor.config(width=frame_cv2.shape[1], height=frame_cv2.shape[0])
                    self.frame_2.pack()
                    self.monitor_know_its_size = True
                    # self.monitor.
                self.frame_2.update_idletasks()
                self.frame_2.update()

    def Check_Path(self):
        self.monitor_know_its_size = False
        self.capture_count = 0
        self.saved_path_to = self.input_path_to.get()
        if os.path.exists(self.saved_path_to):
            shutil.rmtree(self.saved_path_to)
            os.mkdir(self.saved_path_to)
        else:
            os.mkdir(self.saved_path_to)
        self.OpenCamera()

    def GetCamera_ID(self):
        camera_id = self.input_camera_selected.get()
        print('DEBUG: camera_id ', camera_id)
        cameras = camera_id.split(',')
        cameras = [int(dd) for dd in cameras]
        self.camera_used = cameras
        print('DEBUG: cameras ', self.camera_used)
        chess_info = self.input_chess_info.get().split(',')
        self.chess_info = [int(dd) for dd in chess_info]
        print('DEBUG chess_info ', self.chess_info)

    def Run(self):
        self.window.mainloop()


if __name__ == '__main__':
    window = TKWindow()
    window.Run()