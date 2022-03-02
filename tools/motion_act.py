# -*- coding: utf-8 -*-
# only usefull for my robot. define and build some action for it.
import time


class SteeringMoto:
    # steering class, make steer runs simple
    # inv clock is the positive direction
    def __init__(self,
                 device_id=0,
                 P_MAX=2500,
                 P_MIN=500,
                 P_LIMIT_MAX=2500,
                 P_LIMIT_MIN=500,
                 plus_direction=1,
                 roll_range=270):
        # self.mode = mode
        self.device_id = device_id
        self.P_MAX = P_MAX
        self.P_MIN = P_MIN
        # force limit position range. for arm safe
        self.P_LIMIT_MAX = P_LIMIT_MAX
        self.P_LIMIT_MIN = P_LIMIT_MIN
        # -1 for keep direction with clock, 1 for against clock when P get up
        # for mode 1\3\5\7, plus_direction = 1
        self.plus_direction = plus_direction
        self.roll_range = roll_range
        self.command_head = '#' + str(self.device_id).zfill(3) + 'P'
        self.delta_p_per_degree = (self.P_MAX - self.P_MIN) / self.roll_range * self.plus_direction

    def GetRunAngleCommand(self, angle, current, time_t):
        # angle: delta_angle
        # turn follow clock angle = -, else = +
        posi = int(angle * self.delta_p_per_degree + current)
        if posi > self.P_LIMIT_MAX:
            posi = self.P_LIMIT_MAX
        if posi < self.P_LIMIT_MIN:
            posi = self.P_LIMIT_MIN

        command_str = self.command_head + str(posi).zfill(4) + 'T' + str(time_t).zfill(4) + '!'

        return command_str, posi

    def DecodeCurrPosiCommand(self, command_return):
        # #000P1500!
        posi = int(command_return[5:9])
        return posi

    def GetReadPositionCommand(self):
        return self.command_head + 'RAD!'


class RobotArm:
    def __init__(self):
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
        self.joint_0_diverge_angle = 0
        self.joint_1_diverge_angle = 0
        self.joint_2_diverge_angle = 0
        self.joint_3_diverge_angle = 0
        self.joint_4_diverge_angle = 0
        self.joint_5_diverge_angle = 0

    def UpdateSteerPositionBySerialReturn(self, message):
        messages = message.split('!')
        print('DEBUG UpdateSteerPositionBySerialReturn: ', messages, time.time())
        for msg in messages:
            if msg.startswith('#000P'):
                self.joint_0_position = int(msg[5:9])
                self.joint_0_diverge_angle = 0
            elif msg.startswith('#001P'):
                self.joint_1_position = int(msg[5:9])
                self.joint_1_diverge_angle = 0
            elif msg.startswith('#002P'):
                self.joint_2_position = int(msg[5:9])
                self.joint_2_diverge_angle = 0
            elif msg.startswith('#003P'):
                self.joint_3_position = int(msg[5:9])
                self.joint_3_diverge_angle = 0
            elif msg.startswith('#004P'):
                self.joint_4_position = int(msg[5:9])
                self.joint_4_diverge_angle = 0
            elif msg.startswith('#005P'):
                self.joint_5_position = int(msg[5:9])
                self.joint_5_diverge_angle = 0
        print('ROBOT_ARM POSI: 0: ', self.joint_0_position, ' 1: ', self.joint_1_position, ' 2: ',
              self.joint_2_position, ' 3: ', self.joint_3_position, ' 4: ', self.joint_4_position, ' 5: ',
              self.joint_5_position)
        return

    def PitchUp(self, delta_x, time_t=2000):
        self.joint_3_diverge_angle = self.joint_3_diverge_angle - delta_x
        return self.joint_3.GetRunAngleCommand(angle=self.joint_3_diverge_angle,
                                               current=self.joint_3_position,
                                               time_t=time_t)[0]

    def YawUp(self, delta_x, time_t=2000):
        self.joint_0_diverge_angle = self.joint_0_diverge_angle - delta_x
        return self.joint_0.GetRunAngleCommand(angle=self.joint_0_diverge_angle,
                                               current=self.joint_0_position,
                                               time_t=time_t)[0]

    def RollUp(self, delta_x, time_t=2000):
        self.joint_4_diverge_angle = self.joint_4_diverge_angle + delta_x
        return self.joint_4.GetRunAngleCommand(angle=self.joint_4_diverge_angle,
                                               current=self.joint_4_position,
                                               time_t=time_t)[0]

    def StandUp(self, delta_x, time_t=2000):
        self.joint_1_diverge_angle = self.joint_1_diverge_angle - delta_x
        self.joint_2_diverge_angle = self.joint_2_diverge_angle - delta_x * 2
        self.joint_3_diverge_angle = self.joint_3_diverge_angle + delta_x
        return self.joint_1.GetRunAngleCommand(
            angle=self.joint_1_diverge_angle, current=self.joint_1_position,
            time_t=time_t)[0] + self.joint_2.GetRunAngleCommand(
                angle=self.joint_2_diverge_angle, current=self.joint_2_position,
                time_t=time_t)[0] + self.joint_3.GetRunAngleCommand(
                    angle=self.joint_3_diverge_angle, current=self.joint_3_position, time_t=time_t)[0]
