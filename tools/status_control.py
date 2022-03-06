# -*- coding: utf-8 -*-
import time
from motion_act import RobotArm, SteeringMoto
from wheels import Wheels


class MotionStatus:
    def __init__(self, robot_foot):
        self.joint_0 = SteeringMoto(device_id=0,
                                    P_MAX=2500,
                                    P_MIN=500,
                                    P_LIMIT_MAX=2500,
                                    P_LIMIT_MIN=500,
                                    plus_direction=1,
                                    roll_range=270)
        self.joint_1 = SteeringMoto(device_id=1,
                                    P_MAX=2500,
                                    P_MIN=500,
                                    P_LIMIT_MAX=2500,
                                    P_LIMIT_MIN=500,
                                    plus_direction=1,
                                    roll_range=180)
        self.joint_2 = SteeringMoto(device_id=2,
                                    P_MAX=2500,
                                    P_MIN=500,
                                    P_LIMIT_MAX=2500,
                                    P_LIMIT_MIN=500,
                                    plus_direction=1,
                                    roll_range=270)
        self.joint_3 = SteeringMoto(device_id=3,
                                    P_MAX=2500,
                                    P_MIN=500,
                                    P_LIMIT_MAX=2500,
                                    P_LIMIT_MIN=500,
                                    plus_direction=1,
                                    roll_range=270)
        self.joint_4 = SteeringMoto(device_id=4,
                                    P_MAX=2500,
                                    P_MIN=500,
                                    P_LIMIT_MAX=2500,
                                    P_LIMIT_MIN=500,
                                    plus_direction=1,
                                    roll_range=270)
        self.joint_5 = SteeringMoto(device_id=5,
                                    P_MAX=2500,
                                    P_MIN=500,
                                    P_LIMIT_MAX=2500,
                                    P_LIMIT_MIN=500,
                                    plus_direction=1,
                                    roll_range=180)

        self.joint_0_position = 1500
        self.joint_1_position = 1500
        self.joint_2_position = 1500
        self.joint_3_position = 1500
        self.joint_4_position = 1500
        self.joint_5_position = 1500
        self.joint_0_diverge_pwm = 0
        self.joint_1_diverge_pwm = 0
        self.joint_2_diverge_pwm = 0
        self.joint_3_diverge_pwm = 0
        self.joint_4_diverge_pwm = 0
        self.joint_5_diverge_pwm = 0
        self.joint_0_diverge_angle = 0
        self.joint_1_diverge_angle = 0
        self.joint_2_diverge_angle = 0
        self.joint_3_diverge_angle = 0
        self.joint_4_diverge_angle = 0
        self.joint_5_diverge_angle = 0

        self.robot_foot = robot_foot

        self.pre_joint_0_pos = 0
        self.pre_joint_1_pos = 0
        self.pre_joint_2_pos = 0
        self.pre_joint_3_pos = 0
        self.pre_joint_4_pos = 0
        self.pre_joint_5_pos = 0

        self.pre_euler_x = 0
        self.pre_euler_y = 0
        self.pre_euler_z = 0
        self.pre_shift_x = 0
        self.pre_shift_y = 0
        self.pre_shift_z = 0

        self.curr_joint_0_pos = 0
        self.curr_joint_1_pos = 0
        self.curr_joint_2_pos = 0
        self.curr_joint_3_pos = 0
        self.curr_joint_4_pos = 0
        self.curr_joint_5_pos = 0

        self.curr_euler_x = 0
        self.curr_euler_y = 0
        self.curr_euler_z = 0
        self.curr_shift_x = 0
        self.curr_shift_y = 0
        self.curr_shift_z = 0

        self.joint_0_pre_delta = 0
        self.joint_1_pre_delta = 0
        self.joint_2_pre_delta = 0
        self.joint_3_pre_delta = 0
        self.joint_4_pre_delta = 0
        self.joint_5_pre_delta = 0

        self.x_pre_delta = 0
        self.y_pre_delta = 0
        self.z_pre_delta = 0
        return

    def UpdateCurrPwmFromSerial(self, message):
        messages = message.split('!')
        print('DEBUG UpdateCurrPosFromSerial: ', messages, time.time())
        for msg in messages:
            if msg.startswith('#000P'):
                self.joint_0_diverge_pwm = int(msg[5:9]) - self.joint_0_position
                self.joint_0_diverge_angle = (self.joint_0_diverge_pwm) / self.joint_0.delta_p_per_degree
            elif msg.startswith('#001P'):
                self.joint_1_diverge_pwm = int(msg[5:9]) - self.joint_1_position
                self.joint_1_diverge_angle = (self.joint_1_diverge_pwm) / self.joint_1.delta_p_per_degree
            elif msg.startswith('#002P'):
                self.joint_2_diverge_pwm = int(msg[5:9]) - self.joint_2_position
                self.joint_2_diverge_angle = (self.joint_2_diverge_pwm) / self.joint_2.delta_p_per_degree
            elif msg.startswith('#003P'):
                self.joint_3_diverge_pwm = int(msg[5:9]) - self.joint_3_position
                self.joint_3_diverge_angle = (self.joint_3_diverge_pwm) / self.joint_3.delta_p_per_degree
            elif msg.startswith('#004P'):
                self.joint_4_diverge_pwm = int(msg[5:9]) - self.joint_4_position
                self.joint_4_diverge_angle = (self.joint_4_diverge_pwm) / self.joint_4.delta_p_per_degree
            elif msg.startswith('#005P'):
                self.joint_5_diverge_pwm = int(msg[5:9]) - self.joint_5_position
                self.joint_5_diverge_angle = (self.joint_5_diverge_pwm) / self.joint_5.delta_p_per_degree
        print('DEBUG joint angles: ', self.joint_0_diverge_angle, ' [1:]', self.joint_1_diverge_angle, ' [2:]',
              self.joint_2_diverge_angle, ' [3:]', self.joint_3_diverge_angle, ' [4:]', self.joint_4_diverge_angle,
              ' [5:]', self.joint_5_diverge_angle)

    def PitchUp(self, delta_x, time_t=1000):
        self.joint_3_diverge_angle = self.joint_3_diverge_angle - delta_x
        if self.joint_3_diverge_angle > self.joint_3.roll_range / 2:
            self.joint_3_diverge_angle = self.joint_3.roll_range / 2
        if self.joint_3_diverge_angle < self.joint_3.roll_range / -2:
            self.joint_3_diverge_angle = self.joint_3.roll_range / -2
        return self.joint_3.GetRunAngleCommand(angle=self.joint_3_diverge_angle,
                                               current=self.joint_3_position,
                                               time_t=time_t)[0]

    def RollUp(self, delta_x, time_t=1000):
        self.joint_4_diverge_angle = self.joint_4_diverge_angle + delta_x
        if self.joint_4_diverge_angle > self.joint_4.roll_range / 2:
            self.joint_4_diverge_angle = self.joint_4.roll_range / 2
        if self.joint_4_diverge_angle < self.joint_4.roll_range / -2:
            self.joint_4_diverge_angle = self.joint_4.roll_range / -2
        return self.joint_4.GetRunAngleCommand(angle=self.joint_4_diverge_angle,
                                               current=self.joint_4_position,
                                               time_t=time_t)[0]

    def YawUp(self, delta_x, time_t=1000):
        if abs(self.joint_0_diverge_angle) < abs(self.joint_0_diverge_angle - delta_x) and abs(
                self.joint_0_diverge_angle) > 10:
            # use moto wheel turn
            return self.robot_foot.TurnRight(speed=delta_x / abs(delta_x) * 350, time_run=30)
        else:
            self.joint_0_diverge_angle = self.joint_0_diverge_angle - delta_x
            if self.joint_0_diverge_angle > self.joint_0.roll_range / 2:
                self.joint_0_diverge_angle = self.joint_0.roll_range / 2
            if self.joint_0_diverge_angle < self.joint_0.roll_range / -2:
                self.joint_0_diverge_angle = self.joint_0.roll_range / -2
            return self.joint_0.GetRunAngleCommand(angle=self.joint_0_diverge_angle,
                                                   current=self.joint_0_position,
                                                   time_t=time_t)[0]

    def StandUp(self, delta_x, time_t=1000):
        joint1_tmp = self.joint_1_diverge_angle - delta_x
        joint2_tmp = self.joint_2_diverge_angle - delta_x * 2
        joint3_tmp = self.joint_3_diverge_angle + delta_x
        if (joint1_tmp > self.joint_1.roll_range / 2) or (joint1_tmp < self.joint_1.roll_range / -2) or (
                joint2_tmp > self.joint_2.roll_range / 2) or (joint2_tmp < self.joint_2.roll_range / -2) or (
                    joint3_tmp > self.joint_3.roll_range / 2) or (joint3_tmp < self.joint_3.roll_range / -2):
            print('DEBUG Stand Out Of Range')
            return ''
        self.joint_1_diverge_angle = joint1_tmp
        self.joint_2_diverge_angle = joint2_tmp
        self.joint_3_diverge_angle = joint3_tmp
        return self.joint_1.GetRunAngleCommand(
            angle=self.joint_1_diverge_angle, current=self.joint_1_position,
            time_t=time_t)[0] + self.joint_2.GetRunAngleCommand(
                angle=self.joint_2_diverge_angle, current=self.joint_2_position,
                time_t=time_t)[0] + self.joint_3.GetRunAngleCommand(
                    angle=self.joint_3_diverge_angle, current=self.joint_3_position, time_t=time_t)[0]

    def ConditionReflex(self, euler, shift, time_t=1000, **kwargs):
        return self.RollUp(delta_x=euler[2] / 2, time_t=time_t) + self.YawUp(
            delta_x=euler[1] / 2, time_t=time_t) + self.PitchUp(delta_x=euler[0] / 2, time_t=time_t)
        if abs(euler[0]) > abs(euler[1]) and abs(euler[0]) > abs(euler[2]):
            # euler x most
            print('DEBUG Pitch turn')
            return self.PitchUp(delta_x=euler[0] / 2, time_t=time_t)
        elif abs(euler[1]) > abs(euler[0]) and abs(euler[1]) > abs(euler[2]):
            # euler y most
            print('DEBUG Yaw turn')
            return self.YawUp(delta_x=euler[1] / 2, time_t=time_t)
        elif abs(euler[2]) > abs(euler[1]) and abs(euler[2]) > abs(euler[0]):
            # euler z most
            print('DEBUG RollUp turn')
            return self.RollUp(delta_x=euler[2] / 2, time_t=time_t)
