from sensor_msgs.msg import JointState
from typing import List
import rospy
import sys
import os
from math import pi
import psutil
import re
import time
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

if True:
    from dxl.controller import servo_controller

ARM_INITIAL_POSITION = np.array([3352, 3581]) * 360 / 4096
HAND_INITIAL_POSITION = np.array([2800, 2400, 2000, 2900, 3250, 3000]) * 360 / 4096
# INITIAL_POSITION=[0, 0]
LOWER_ARM_MAPPING = {7: "l_hand_yaw", 8: "l_hand_roll"}


class LowerArmController:
    sc: servo_controller
    last_command: List[int] = [0] * 8

    def __init__(self, hand_only=False) -> None:
        # self.sc = servo_controller([list(range(7, 9)), list(range(17, 21))],
        self.sc = servo_controller(
            [list(range(1, 7)), list(range(7, 9))],
            BAUDRATE=2e6,
            DEVICE_NAME="/dev/ttyUSB0",
            hand_only=hand_only,
        )

    def init_motor_controller(self):
        self.sc.enable()
        self.sc.change_controller_p_value_each(400, 1)

    def handle_hand_service(self, hand_config):
        pass

    def pose_init(self):
        self.last_command = list(HAND_INITIAL_POSITION) + list(ARM_INITIAL_POSITION)
        # self.last_command = list(ARM_INITIAL_POSITION)
        self.sc.move_to_goal_position(goal_position=self.last_command, step_raleted=100)
        print(self.last_command)
        # [207.59765625, 265.166015625, 287.490234375, 270.087890625, 286.787109375, 72.685546875, 72.509765625, 270.087890625]

    def arm_config_callback(self, state: JointState):
        joint_state_dict = dict(zip(state.name, state.position))
        arm_config = [
            joint_state_dict.get(key, 0) for key in LOWER_ARM_MAPPING.values()
        ]
        # print("arm config: ", arm_config)
        for i in range(2):
            self.last_command[i + 6] = (
                (180 / pi) * arm_config[i]
            ) + ARM_INITIAL_POSITION[i]
        self.sc.goto_angle(self.last_command, no_sleep=True)

    def hand_config_callback(self, state: JointState):
        hand_config = self.create_hand_config(state)
        for i in range(6):
            self.last_command[i] = hand_config[i]
        rospy.loginfo("hand config callback triggered: ")
        print(self.last_command)
        self.sc.move_to_goal_position(self.last_command, step_raleted=80)

    # def execute_joint_state(self, joint_state: JointState):
    #     # joint_state to dict
    #     hand_config = self.create_hand_config(joint_state)
    #     if any(hand_config):
    #         for i in range(6):
    #             self.last_command[i] = hand_config[i]
    #     joint_state_dict = dict(zip(joint_state.name, joint_state.position))
    #     arm_config = [
    #         joint_state_dict.get(key, 0) for key in LOWER_ARM_MAPPING.values()
    #     ]
    #     # print("arm config: ", arm_config)
    #     if any(arm_config):
    #         for i in range(2):
    #             self.last_command[i + 6] = (
    #                 (180 / pi) * arm_config[i]
    #             ) + ARM_INITIAL_POSITION[i]
    #     print(arm_config)
    #     print(self.last_command)
    #     if not any(hand_config) and not any(arm_config):
    #         return
    #     # # get goal position
    #     if not any(arm_config):
    #         # hand config only
    #         self.sc.move_to_goal_position(self.last_command)
    #     else:
    #         self.sc.goto_angle(self.last_command, no_sleep=True)

    def create_hand_config(self, hand_config: JointState):
        thumb_oppo, thumb_value, index_value, middle_value, ring_value, little_value = (
            0,
            0,
            0,
            0,
            0,
            0,
        )
        for i, name in enumerate(hand_config.name):
            if name == "lh_thumb1":
                thumb_oppo = hand_config.position[i]
            elif re.search(r"thumb[23]", name):
                thumb_value += hand_config.position[i]
            elif name.find("index") != -1:
                index_value += hand_config.position[i]
            elif name.find("middle") != -1:
                middle_value += hand_config.position[i]
            elif name.find("ring") != -1:
                ring_value += hand_config.position[i]
            elif name.find("little") != -1:
                little_value += hand_config.position[i]
        if not any(
            [
                thumb_oppo,
                thumb_value,
                index_value,
                middle_value,
                ring_value,
                little_value,
            ]
        ):
            return [
                thumb_oppo,
                thumb_value,
                index_value,
                middle_value,
                ring_value,
                little_value,
            ]

        # [147.3046875, 265.166015625, 199.599609375, 270.087890625, 286.787109375, 72.685546875, 72.509765625, 270.087890625]
        thumb_oppo = int(thumb_oppo * 360 / (2 * pi))
        thumb_value = int(thumb_value * 360 / (2 * pi) / 2.1)
        index_value = int(index_value * 360 / (2 * pi) / 2.1)  # inverse
        middle_value = int(middle_value * 360 / (2 * pi) / 2.1)
        ring_value = int(ring_value * 360 / (2 * pi) / 2.1)
        little_value = int(little_value * 360 / (2 * pi) / 2.1)
        goal_position = list(
            np.array(
                [
                    thumb_oppo,
                    thumb_value,
                    index_value,
                    middle_value,
                    ring_value,
                    little_value,
                ]
            )
            + HAND_INITIAL_POSITION
        )
        return goal_position


def main():
    rospy.init_node("lower_arm_node", anonymous=True)
    arm_controller = LowerArmController()
    arm_controller.init_motor_controller()
    arm_controller.pose_init()
    subscriber = rospy.Subscriber(
        "/arm_command_desired",
        JointState,
        arm_controller.arm_config_callback,
        queue_size=10,
    )
    subscriber_hand = rospy.Subscriber(
        "/hand_config_desired",
        JointState,
        arm_controller.hand_config_callback,
        queue_size=1,
    )
    # rospy.Subscriber("/hand_config_desired",  JointState,
    #                  hand_controller.command_callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    print(os.sched_get_priority_max(os.SCHED_FIFO))
    param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    os.sched_setscheduler(0, os.SCHED_FIFO, param)
    main()
