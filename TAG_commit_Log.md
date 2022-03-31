
commit c844355d1097f24b38c46ae9d2f4dc85a1d05bc0 (HEAD -> main)
Author: notebook <frankensteinquasimo@outlook.com>
Date:   Thu Mar 31 19:03:27 2022 +0800
    add server agency
describe:
    add server_agency. because raspberry's cpu is so slow


commit 305bc12f9e586cdad713be921f637a0f1806a8d2 (HEAD -> main)
Author: notebook <frankensteinquasimo@outlook.com>
Date:   Sun Mar 27 04:27:19 2022 +0800
    add serial support with ~ unibusgit add tools/robo_serial.py
describe:
    the control board is designed werid, serial RXD & TXD share the same line. In order to escape the conflict between RXD and TXD, the "self.unibus" was required when Serial "__init__".


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