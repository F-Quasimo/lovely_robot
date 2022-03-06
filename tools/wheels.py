# -*- coding: utf-8 -*-
# only usefull for my robot. define and build some action for it.
from distutils import command
import time
from tkinter import LEFT


class Wheel:
    def __init__(self,
                 moto_id,
                 stop_speed=1500,
                 direction=1,
                 position='LF',
                 open_or_close='open',
                 range_min=500,
                 range_max=2500):
        self.moto_id = moto_id
        self.direction = direction
        self.open_or_close = open_or_close
        self.position = position
        self.init_run = True
        self.init_run_commands = ''
        self.stop_speed = stop_speed
        self.command_head = '#' + str(int(self.moto_id)).zfill(3) + 'P'
        self.range_min = range_min
        self.range_max = range_max
        if self.open_or_close == 'open':
            self.init_run_commands = self.command_head + 'OPN!'
        else:
            self.init_run_commands = self.command_head + 'CLS!'

    def Run(self, speed, time_run=1000):
        pwm_num = speed * self.direction + self.stop_speed
        if pwm_num > self.range_max:
            pwm_num = self.range_max
        elif pwm_num < self.range_min:
            pwm_num = self.range_min
        command_out = self.command_head + str(int(pwm_num)).zfill(4) + 'T' + str(int(time_run)).zfill(4) + '!'
        if self.init_run == True:
            return command_out
        else:
            self.init_run = True
            return self.init_run_commands + command_out


class Wheels:
    def __init__(self):
        # params not open now
        # Left/Right + Front/Rear
        self.left_front_wheel = Wheel(moto_id=6,
                                      stop_speed=1500,
                                      direction=1.0,
                                      position='LF',
                                      open_or_close='open',
                                      range_max=2500,
                                      range_min=500)
        self.right_front_wheel = Wheel(moto_id=7,
                                       stop_speed=1500,
                                       direction=-1.0,
                                       position='RF',
                                       open_or_close='open',
                                       range_max=2500,
                                       range_min=500)
        self.left_rear_wheel = Wheel(moto_id=8,
                                     stop_speed=1500,
                                     direction=1,
                                     position='LR',
                                     open_or_close='open',
                                     range_max=2500,
                                     range_min=500)
        self.right_rear_wheel = Wheel(moto_id=9,
                                      stop_speed=1500,
                                      direction=-1.0,
                                      position='RR',
                                      open_or_close='open',
                                      range_max=2500,
                                      range_min=500)

    def Run(self, left_front, right_front, left_rear, right_rear, time_run=1000):
        command_out = self.left_front_wheel.Run(speed=left_front, time_run=time_run) + self.right_front_wheel.Run(
            speed=right_front, time_run=time_run) + self.left_rear_wheel.Run(
                speed=left_rear, time_run=time_run) + self.right_rear_wheel.Run(speed=right_rear, time_run=time_run)
        return command_out

    def LeftTranslation(self, speed=300, time_run=1000):
        return self.Run(left_front=-1 * speed,
                        right_front=speed,
                        left_rear=speed,
                        right_rear=-1 * speed,
                        time_run=time_run)

    def FrontTranslation(self, speed=300, time_run=1000):
        return self.Run(left_front=speed, right_front=speed, left_rear=speed, right_rear=speed, time_run=time_run)

    def RightTranslation(self, speed, time_run=1000):
        return self.LeftTranslation(speed=speed * -1, time_run=time_run)

    def BackTranslation(self, speed, time_run=1000):
        return self.FrontTranslation(speed=speed * -1, time_run=time_run)

    def TurnLeft(self, speed=300, time_run=1000):
        return self.Run(left_front=-1 * speed,
                        left_rear=-1 * speed,
                        right_front=speed,
                        right_rear=speed,
                        time_run=time_run)

    def TurnRight(self, speed, time_run=1000):
        return self.TurnLeft(speed=-1 * speed, time_run=time_run)

    def WheelYawUp(self, speed=300, time_run=1000):
        return self.Run(left_front=0, left_rear=speed, right_front=0, right_rear=-1 * speed, time_run=time_run)
