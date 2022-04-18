import ncnn
import os
import cv2

# Tencent is pleased to support the open source community by making ncnn available.
#
# Copyright (C) 2020 THL A29 Limited, a Tencent company. All rights reserved.
#
# Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
# in compliance with the License. You may obtain a copy of the License at
#
# https://opensource.org/licenses/BSD-3-Clause
#
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.


class Point(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0


class KeyPoint(object):
    def __init__(self):
        self.p = Point()
        self.prob = 0.0


exp_dir = os.path.dirname(__file__)


def draw_pose(image, keypoints):
    # draw bone
    joint_pairs = [
        (0, 1),
        (1, 3),
        (0, 2),
        (2, 4),
        (5, 6),
        (5, 7),
        (7, 9),
        (6, 8),
        (8, 10),
        (5, 11),
        (6, 12),
        (11, 12),
        (11, 13),
        (12, 14),
        (13, 15),
        (14, 16),
    ]

    for i in range(16):
        p1 = keypoints[joint_pairs[i][0]]
        p2 = keypoints[joint_pairs[i][1]]

        if p1.prob < 0.2 or p2.prob < 0.2:
            continue

        cv2.line(
            image,
            (int(p1.p.x), int(p1.p.y)),
            (int(p2.p.x), int(p2.p.y)),
            (255, 0, 0),
            3,
        )

    # draw joint
    for keypoint in keypoints:
        print("%.2f %.2f = %.5f" % (keypoint.p.x, keypoint.p.y, keypoint.prob))

        if keypoint.prob < 0.2:
            continue

        cv2.circle(image, (int(keypoint.p.x), int(
            keypoint.p.y)), 3, (0, 255, 0), -1)

    return image


class PersonDetect:
    def __init__(self, use_gpu=False, threads=4, param_path=os.path.join(exp_dir, 'checkpoints/person_detector.param'),
                 bin_path=os.path.join(exp_dir, 'checkpoints/person_detector.bin')):
        self.param_path = param_path
        self.bin_path = bin_path
        # use_gpu=ncnn.get_gpu_count()>0 and use_gpu
        self.use_gpu = use_gpu
        self.norm_vals = [1 / 255.0, 1 / 255.0, 1 / 255.0]
        self.mean_vals = [0, 0, 0]
        self.num_threads = threads
        self.detector_size_width = 320
        self.detector_size_height = 320
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = self.use_gpu
        self.net.load_param(self.param_path)
        self.net.load_model(self.bin_path)

    def __del__(self):
        self.net = None

    def __call__(self, img):
        img_h = img.shape[0]
        img_w = img.shape[1]

        mat_in = ncnn.Mat.from_pixels_resize(
            img,
            ncnn.Mat.PixelType.PIXEL_BGR2RGB,
            img.shape[1],
            img.shape[0],
            self.detector_size_width,
            self.detector_size_height,
        )
        mat_in.substract_mean_normalize(self.mean_vals, self.norm_vals)

        ex = self.net.create_extractor()
        ex.set_light_mode(True)
        ex.set_num_threads(self.num_threads)

        ex.input("data", mat_in)

        ret, mat_out = ex.extract("output")

        for idx in range(mat_out.h):
            values = mat_out.row(idx)
            x1 = values[2]*img_w
            y1 = values[3]*img_h
            x2 = values[4]*img_w
            y2 = values[5]*img_h

            pw = x2 - x1
            ph = y2 - y1
            cx = x1 + 0.5 * pw
            cy = y1 + 0.5 * ph

            if pw/192.0 > ph/256.0:
                # pw prefer
                x1 = cx-0.5*pw
                x2 = cx+0.5*pw
                y1 = cy-pw/192.0*256/2
                y2 = cy+pw/192.0*256/2
            else:
                y1 = cy-0.5*ph
                y2 = cy+0.5*ph
                x1 = cx-ph/256.0*192/2
                x2 = cx+ph/256.0*192/2
            x1 = cx - 0.7 * pw
            y1 = cy - 0.6 * ph
            x2 = cx + 0.7 * pw
            y2 = cy + 0.6 * ph

            if x1 < 0:
                x1 = 0
            if x2 < 0:
                x2 = 0
            if y1 < 0:
                y1 = 0
            if y2 < 0:
                y2 = 0
            if x1 > img_w:
                x1 = img_w
            if x2 > img_w:
                x2 = img_w
            if y1 > img_h:
                y1 = img_h
            if y2 > img_h:
                y2 = img_h

            score = values[1]
            label = values[0]

            return img[int(y1):int(y2), int(x1):int(x2)]


class SimplePose:
    def __init__(
        self, target_width=192, target_height=256, num_threads=4, use_gpu=False,
        param_path=os.path.join(
            exp_dir, 'checkpoints/Ultralight-Nano-SimplePose.param'),
        bin_path=os.path.join(
            exp_dir, 'checkpoints/Ultralight-Nano-SimplePose.bin')
    ):
        self.target_width = target_width
        self.target_height = target_height
        self.num_threads = num_threads
        self.use_gpu = use_gpu

        self.mean_vals = [0.485 * 255.0, 0.456 * 255.0, 0.406 * 255.0]
        self.norm_vals = [1 / 0.229 / 255.0,
                          1 / 0.224 / 255.0, 1 / 0.225 / 255.0]

        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = self.use_gpu
        self.net.opt.use_fp16_arithmetic = True
        # the simple baseline human pose estimation from gluon-cv
        # https://gluon-cv.mxnet.io/build/examples_pose/demo_simple_pose.html
        # mxnet model exported via
        #      pose_net.hybridize()
        #      pose_net.export('pose')
        # then mxnet2ncnn
        # the ncnn model https://github.com/nihui/ncnn-assets/tree/master/models
        self.net.load_param(param_path)
        self.net.load_model(bin_path)

    def __del__(self):
        self.net = None

    def __call__(self, img):
        h = img.shape[0]
        w = img.shape[1]
        # img=cv2.resize(img,(192,256))
        mat_in = ncnn.Mat.from_pixels_resize(
            img,
            ncnn.Mat.PixelType.PIXEL_BGR2RGB,
            img.shape[1],
            img.shape[0],
            self.target_width,
            self.target_height,
        )
        mat_in.substract_mean_normalize(self.mean_vals, self.norm_vals)
        ex = self.net.create_extractor()
        ex.set_light_mode(True)
        ex.set_num_threads(self.num_threads)

        ex.input("data", mat_in)

        # ret, mat_out = ex.extract("conv3_fwd")
        ret, mat_out = ex.extract("hybridsequential0_conv7_fwd")

        keypoints = []

        for p in range(mat_out.c):
            m = mat_out.channel(p)

            max_prob = 0.0
            max_x = 0
            max_y = 0
            for y in range(mat_out.h):
                ptr = m.row(y)
                for x in range(mat_out.w):
                    prob = ptr[x]
                    if prob > max_prob:
                        max_prob = prob
                        max_x = x
                        max_y = y

            keypoint = KeyPoint()
            keypoint.p.x = max_x * w / float(mat_out.w)
            keypoint.p.y = max_y * h / float(mat_out.h)
            keypoint.prob = max_prob

            keypoints.append(keypoint)
        if len(keypoints) > 0:
            drawed = draw_pose(img, keypoints)
            return keypoints, drawed
        else:
            return None, None


class SimplePose2:
    def __init__(
        self, target_width=192, target_height=256, num_threads=1, use_gpu=False,
        param_path=os.path.join(
            exp_dir, 'checkpoints/pose.param'),
        bin_path=os.path.join(
            exp_dir, 'checkpoints/pose.bin')
    ):
        self.target_width = target_width
        self.target_height = target_height
        self.num_threads = num_threads
        self.use_gpu = use_gpu

        self.mean_vals = [0.485 * 255.0, 0.456 * 255.0, 0.406 * 255.0]
        self.norm_vals = [1 / 0.229 / 255.0,
                          1 / 0.224 / 255.0, 1 / 0.225 / 255.0]

        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = self.use_gpu

        # the simple baseline human pose estimation from gluon-cv
        # https://gluon-cv.mxnet.io/build/examples_pose/demo_simple_pose.html
        # mxnet model exported via
        #      pose_net.hybridize()
        #      pose_net.export('pose')
        # then mxnet2ncnn
        # the ncnn model https://github.com/nihui/ncnn-assets/tree/master/models
        self.net.load_param(param_path)
        self.net.load_model(bin_path)

    def __del__(self):
        self.net = None

    def __call__(self, img):
        h = img.shape[0]
        w = img.shape[1]
        img_resize = cv2.resize(img, (192, 256))
        mat_in = ncnn.Mat.from_pixels_resize(
            img_resize,
            ncnn.Mat.PixelType.PIXEL_BGR2RGB,
            img.shape[1],
            img.shape[0],
            self.target_width,
            self.target_height,
        )
        mat_in.substract_mean_normalize(self.mean_vals, self.norm_vals)

        ex = self.net.create_extractor()
        ex.set_num_threads(self.num_threads)
        ex.input("data", mat_in)

        ret, mat_out = ex.extract("conv3_fwd")

        keypoints = []

        for p in range(mat_out.c):
            m = mat_out.channel(p)

            max_prob = 0.0
            max_x = 0
            max_y = 0
            for y in range(mat_out.h):
                ptr = m.row(y)
                for x in range(mat_out.w):
                    prob = ptr[x]
                    if prob > max_prob:
                        max_prob = prob
                        max_x = x
                        max_y = y

            keypoint = KeyPoint()
            keypoint.p.x = max_x * w / float(mat_out.w)
            keypoint.p.y = max_y * h / float(mat_out.h)
            keypoint.prob = max_prob

            keypoints.append(keypoint)

        if len(keypoints) > 0:
            drawed = draw_pose(img, keypoints)
            return keypoints, drawed
        else:
            return None, None
