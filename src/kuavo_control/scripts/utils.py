#!/usr/bin/env python3

import rospy
import json
import math
import os
import moveit_msgs.msg
import rospy_message_converter.json_message_converter


def load_config(path: str) -> dict:
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "r") as f:
        json_data = json.load(f)
    rospy.loginfo("配置已从{}中加载".format(path))
    return json_data


def load_joints(path: str) -> dict:
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "r") as f:
        json_data = json.load(f)
    rospy.loginfo("轨迹点已从{}中加载".format(path))
    return json_data


def load_traj(path: str) -> moveit_msgs.msg.RobotTrajectory:
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "r") as f:
        traj = json.load(f)
    traj = rospy_message_converter.json_message_converter.convert_json_to_ros_message("moveit_msgs/RobotTrajectory", traj)
    rospy.loginfo("轨迹已从{}中加载".format(path))
    return traj


def dump_traj(path: str, traj: moveit_msgs.msg.RobotTrajectory) -> None:
    traj = rospy_message_converter.json_message_converter.convert_ros_message_to_json(traj)
    path = os.path.join(os.path.dirname(__file__), path)
    with open(path, "w") as f:
        json.dump(traj, f)
    rospy.loginfo("轨迹已保存到{}".format(path))


def rad_to_angle(rad_list: list) -> list:
    """弧度转变为角度
    """
    angle_list = [0 for _ in range(len(rad_list))]
    for i, rad in enumerate(rad_list):
        angle_list[i] = rad / math.pi * 180
    return angle_list


def l_to_r(position: list) ->list:
    """左手到右手
    """
    position[1] = -position[1]
    position[2] = -position[2]
    position[5] = -position[5]
    return position


class TransformPos():
    pass





