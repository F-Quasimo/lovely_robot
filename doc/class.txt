class Puppy:
    serial_bus = MySerial()
    self.robot_arm = RobotArm()
    self.wheel4 = Wheel4()
    self.bbi = BiBi()
    self.blink = Blink()
    self.cam_mono = SingleCamera()
    self.cam_stereo = StereoCamera()

class MySerial:
    self.sender_queue
    self.receiver_queue
    self.IsBusy

class RobotArm:
    self.joint0 = SteeringMoto()
    self.joint1...

    func

class SteeringMoto:
    self.id
    self.range
    self.position
    ...

class Wheel4:
    self.wheel0 = Wheel()
    self.wheel1 = Wheel()
    ...

    def func

class Wheel:
    self.id
    self.posi = LF RF LB RB

    ...

class SingleCamera:
    self.calib_...
    self.camera_info

    def Snap(undistort)
    def func

class StereoCamera:
    self.calib_...
    self.camera_info

    def func


