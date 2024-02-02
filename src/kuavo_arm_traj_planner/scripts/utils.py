#!/usr/bin/env python3

import rospy
import json
import math
import os
import moveit_msgs.msg
import trajectory_msgs.msg
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


def l_to_r(l_traj: moveit_msgs.msg.RobotTrajectory) -> moveit_msgs.msg.RobotTrajectory:
    """左手到右手轨迹对称映射
    """
    r_traj = moveit_msgs.msg.RobotTrajectory()
    r_traj.joint_trajectory.header = l_traj.joint_trajectory.header
    r_traj.joint_trajectory.joint_names = [
        "r_arm_pitch",
        "r_arm_roll",
        "r_arm_yaw",
        "r_forearm_pitch",
        "r_forearm_yaw",
        "r_hand_roll",
        "r_hand_pitch"
    ]
    
    for l_point in l_traj.joint_trajectory.points:
        r_point = trajectory_msgs.msg.JointTrajectoryPoint()
        r_point.time_from_start = l_point.time_from_start
        r_point.positions = [
             l_point.positions[0],
            -l_point.positions[1],
            -l_point.positions[2],
             l_point.positions[3],
             l_point.positions[4],
            -l_point.positions[5],
             l_point.positions[6]
        ]
        r_point.velocities = l_point.velocities
        r_point.accelerations = l_point.accelerations
        r_traj.joint_trajectory.points.append(r_point)
    
    return r_traj


def calibration_roban(joints: list):
    joints = [
        -(joints[0] - 1.9309829473495483),
        joints[1] - 11.722798347473145,
        joints[2] + 97.14885711669922,
        -(joints[3] + 13.931812286376953),
        joints[4] + 105.603764533996582,
        joints[5] + 98.72225952148438,
        -(joints[6] + 64.80595397949219)
    ]

    for i, joint in  enumerate(joints):
        joints[i] = joint / 180 * math.pi

    return joints

