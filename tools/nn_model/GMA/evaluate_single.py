import sys
import argparse
import os
import cv2
import glob
import numpy as np
import torch
from PIL import Image
import imageio
import matplotlib.pyplot as plt

sys.path.append('core')
# sys.path.append('.')
exp_dir = os.path.dirname(__file__)
sys.path.append(os.path.join(exp_dir, 'core'))
sys.path.append(exp_dir)

from network import RAFTGMA
from utils import flow_viz
from utils.utils import InputPadder
import os

DEVICE = 'cpu'

parser = argparse.ArgumentParser()
parser.add_argument('--model', help="restore checkpoint")
parser.add_argument('--model_name', help="define model name", default="GMA")
parser.add_argument('--path', help="dataset for evaluation")
parser.add_argument('--num_heads', default=1, type=int,
                    help='number of heads in attention and aggregation')
parser.add_argument('--position_only', default=False, action='store_true',
                    help='only use position-wise attention')
parser.add_argument('--position_and_content', default=False, action='store_true',
                    help='use position and content-wise attention')
parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
args = parser.parse_args()

def load_image(imfile):
    img = np.array(Image.open(imfile)).astype(np.uint8)
    img = torch.from_numpy(img).permute(2, 0, 1).float()
    return img[None].to(DEVICE)

def load_image_from_np(img_np):
    img = torch.from_numpy(img_np).permute(2, 0, 1).float()
    return img[None].to(DEVICE)


def viz(img, flo, flow_dir, flow_name):
    img = img[0].permute(1, 2, 0).cpu().numpy()
    flo = flo[0].permute(1, 2, 0).cpu().numpy()

    # map flow to rgb image
    flo = flow_viz.flow_to_image(flo)

    imageio.imwrite(os.path.join(flow_dir, flow_name+'.png'), flo)
    print(f"Saving optical flow visualisation at {os.path.join(flow_dir, 'flo.png')}")


def normalize(x):
    return x / (x.max() - x.min())


def demo(args):
    model = torch.nn.DataParallel(RAFTGMA(args))
    model.load_state_dict(torch.load(args.model, map_location=torch.device('cpu')))
    print(f"Loaded checkpoint at {args.model}")

    model = model.module
    model.to(DEVICE)
    model.eval()

    flow_dir = os.path.join(args.path, args.model_name)
    if not os.path.exists(flow_dir):
        os.makedirs(flow_dir)

    with torch.no_grad():
        images = glob.glob(os.path.join(args.path, '*.png')) + \
                 glob.glob(os.path.join(args.path, '*.jpg'))

        img_main = [dd for dd in images if 'p_main' in dd]
        img_aux = [dd for dd in images if 'p_aux' in dd]
        # images = sorted(images)
        img_main = sorted(img_main)
        img_aux = sorted(img_aux)
        for imfile1, imfile2 in zip(img_main, img_aux):
            image1 = load_image(imfile1)
            image2 = load_image(imfile2)
            flow_name=os.path.split(imfile1)[-1].replace('.jpg','')+'_'+os.path.split(imfile2)[-1].replace('.jpg','')
            print(f"Reading in images at {imfile1} and {imfile2} save :{flow_name}")

            padder = InputPadder(image1.shape)
            image1, image2 = padder.pad(image1, image2)

            flow_low, flow_up = model(image1, image2, iters=12, test_mode=True)
            print(f"Estimating optical flow...")

            viz(image1, flow_up, flow_dir, flow_name)

class GmaFlow:
    def __init__(self, ckpt=os.path.join(exp_dir,'checkpoints/gma-sintel.pth')):
        self.ckpt=ckpt
        self.model = torch.nn.DataParallel(RAFTGMA(args))
        self.model.load_state_dict(torch.load(ckpt, map_location=torch.device('cpu')))
        self.model = self.model.module
        self.model.to(DEVICE)
        self.model.eval()

    def Run(self, img0, img1, iters=12):
        image1 = load_image_from_np(img0)
        image2 = load_image_from_np(img1)
        padder = InputPadder(image1.shape)
        image1, image2 = padder.pad(image1, image2)
        flow_low, flow_up = self.model(image1, image2, iters=iters, test_mode=True)
        flow_low = flow_low[0].permute(1, 2, 0).cpu().detach().numpy()
        flow_up = flow_up[0].permute(1, 2, 0).cpu().detach().numpy()
        return flow_up[0:img0.shape[0],0:img0.shape[1]]


if __name__ == '__main__':

    args.path = r'D:\ubuntu18_win\rootfs\home\fq\lovely_robot\tools\stereo_test'
    # args.model = r'D:\ubuntu18_win\rootfs\home\fq\github\GMA\checkpoints\gma-sintel.pth'
    # demo(args)
