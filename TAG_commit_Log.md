

commit f0f5c11d1aac19c9126a7c2ec137313e1214d63d (HEAD -> main)
Author: notebook <frankensteinquasimo@outlook.com>
Date:   Fri Mar 18 00:18:58 2022 +0800

    raspberry videocapture fix bug
describe:
    raspberry opencamera cant fluent cv2.imshow~,and must add ".set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))"to decode stream. use 
    v4l2-ctl -d /dev/video0 --all
    v4l2-ctl -d /dev/video2 --list-formats
    for more info.
    doesnt matter: set timeoout: in terminal:
    onece: modprobe uvcvideo nodrop=1 timeout=6000
    forever:  modprobe uvcvideo nodrop=1 timeout=6000
    sudo gedit /etc/modprobe.d/modprobe.conf