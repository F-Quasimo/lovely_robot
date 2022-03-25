
-------------------------- 2022-03-22  --------
I refresh my respberry from 32bit os to 64 bit os, which means I will get powerful compute ability and much more lib assistant.
but things gonna be circuitous. There are something that need to be care about:
1) Serial port changes, it use to be /dev/ttyAMA0, Now is /dev/ttyS0
2) stereo camera id is : 0, 2 ??? where cam 1 at?
3) build pytorch 1.12, with NNPACK QNNPACK MKLDNN but it is still slower than pip install 1.11.0, maybe NEON make changes. anyway, it works well now.
4) "pip install torchvision" doesnt work well with "piped install torch" when i run YOLOX demo. so torchvision in env(conda activate py37torch) was build from source.
5) when raspberry's cpu working hard for neural network, i mean, cpu worked under 100% mode. more power input was great. power supplied in usb, YOLOX-tiny may runs 1.2 s / frame, extra power plug in, it can runs twice as fast.
6) raspberry's poor compute ability cant processing video stream realtime, so , a video frame buffer in camera_mode.py maybe great. so, a loop-queue with 3 frames buffer was create.(i am willing to use 3 frames as buffer; but 2 frames is good enough)

我给树莓派刷了64位系统，或许能得到更强的计算资源利用率和更广的算法库支持，但是这个过程有点扯淡。
1）串口号改了，之前是/dev/ttyAMA0而且在64位系统下用库读出来也是这个，但是实际能用的是/dev/ttyS0
2）插双目相机时候设备名分别是0，2
3）pip安装的pytorch比源码编的pytorch更快，即便我使用了NNPACK、QNNPACK和MKLDNN，可能是NEON没开的原因。实际CPU占用率都有100%的
4）pip安装的torchvison在YOLOX里和pip安装的pytorch配合不好，所以torchvision用的源码安装但是在/home/pi/github/vision当日状态下，关闭ffmpeg才能过。
5）cpu疯狂运算时候，usb供电不足，这时候性能下降厉害，需要额外供电，性能能差一倍
6）树莓派弱鸡的算力不足以支撑实时视频流处理，尤其接摄像头时候，为了缓解延迟的问题，我用3帧的循环队列解决了这个问题。(我想的时候是3帧，但2帧差不多得了)
ps:后悔，贫穷，想买jetson nano，哪怕rk3399呢