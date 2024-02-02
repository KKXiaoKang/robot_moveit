#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# BEGIN_SUB_TUTORIAL imports
##
# To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
# This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
# and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports

from __future__ import annotations
import sys
import re
import copy
import rospy
import json
import yaml
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import csv
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rospy_message_converter import json_message_converter

from math import pi, tau, dist, fabs, cos

from typing import Any, Dict, List

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list

# END_SUB_TUTORIAL

ARM_JOINT_CONFIG = [-7, -13, -2, -75, 12, -8]
# ARM_JOINT_CONFIG = [-50, -30, 0, 0, 0, 0]


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class LejuController(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(LejuController, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("leju_robot", anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "left"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.planning_frame = self.move_group.get_planning_frame() #获取运动规划的参考框架

        self.eef_link = self.move_group.get_end_effector_link() #获取末端执行器

        self.group_names = self.robot.get_group_names() #获取分组名称

        self.touch_links_left = [
            "lh_thumb1",
            "lh_thumb2",
            "lh_thumb3",
            "lh_index1",
            "lh_index2",
            "lh_index3",
            "lh_middle1",
            "lh_middle2",
            "lh_middle3",
            "lh_ring1",
            "lh_ring2",
            "lh_ring3",
            "lh_little1",
            "lh_little2",
            "lh_little3",
        ]

        self.touch_links_right = [
            link.replace("lh", "rh") for link in self.touch_links_left
        ]
        self.joint_command_publisher = rospy.Publisher(
            "/joint_command_desired", JointState, queue_size=1
        )

        self.ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.loginfo("Waiting for /compute_ik service...")
        self.ik_srv.wait_for_service()
        self.arm_command_publisher = rospy.Publisher(
            "/arm_command_desired", JointState, queue_size=1
        )
        self.hand_config_publisher = rospy.Publisher(
            "/hand_config_desired", JointState, queue_size=1
        )

        # self.arm_command_publisher = rospy.Publisher("/arm_command_desired", JointState, queue_size=1)
        # self.csv_writer = csv.writer(open("arm_command.csv", "w"))

        # Misc variables
        self.clear_environment() #清除环境中的物体
        self.reset_fingers_sim() #重制手指状态
        # self.add_cylinder()
        self.add_drawer()
        self.add_bottle()

    # def record_arm_command(self, joint_state):
    #     self.csv_writer.writerow(joint_state.position)

    #     if not rospy.is_shutdown():
    #         self.arm_command_publisher.publish(joint_state)

    def test(self):
        # self.add_drawer()
        traj = json_message_converter.convert_json_to_ros_message(
            "moveit_msgs/RobotTrajectory",
            (
                json.dumps(
                    json.load(
                        open(
                            "/home/lab/hand_12/humanoidmanipulation/hand_arm_ws/src/leju_controller/hand_config/traj_py.json",
                            "r",
                        )
                    )
                )
            ),
        )
        hand_config = json_message_converter.convert_json_to_ros_message(
            "sensor_msgs/JointState",
            (
                json.dumps(
                    json.load(
                        open(
                            "/home/lab/hand_12/humanoidmanipulation/hand_arm_ws/src/leju_controller/hand_config/hand_py.json",
                            "r",
                        )
                    )
                )
            ),
        )

        self.execute_plan(traj, hand_config)
        # exit(0)

    def home(self):
        rospy.loginfo("robot going home")
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)

    def get_ik(self, pose):
        """
        逆运动学求解
        
        :param pose: 当前位姿
        .
        :return: 逆运动学求解关节
        """
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = rospy.Time.now()
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = self.robot.get_current_state()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(0, 2e8)
        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp

    def adjust_grasp_pose(self, pose: geometry_msgs.msg.Pose):
        # pose.position.x += 0.36
        # pose.position.y += 0.13
        # pose.position.z += 0.8268
        pose.position.x += 0.28
        pose.position.y += 0.15
        pose.position.z += 0.823
        return pose

    def goto_joint_config(self, joints):
        """
        去往到关节位姿，轨迹存入traj_py.json
        
        :param joints: 去往的关节位姿
        """
        self.move_group.set_planner_id("RRTstar")
        self.move_group.set_joint_value_target(np.array(joints) / 180 * pi)
        result = self.move_group.plan()
        # debug
        print("result:",result)

        (success, traj, *_) = result

        # 重新计时轨迹，可靠平滑
        traj = self.move_group.retime_trajectory(
            self.robot.get_current_state(),
            traj,
            velocity_scaling_factor=0.15,
            acceleration_scaling_factor=0.15,
            algorithm="time_optimal_trajectory_generation",
        )

        text = input("input anything to save, press enter to exit")
        if text:
            print("i:")
            traj_json = json.loads(
                json_message_converter.convert_ros_message_to_json(traj)
            )
            # print("j:",traj_json)
            json.dump(
                traj_json,
                open(
                    "/home/lab/hand_12/humanoidmanipulation/hand_arm_ws/src/leju_controller/hand_config/traj_py.json",
                    "w",
                ),
            )
            # exit(0)

    def goto_hand_pose(self, json_file: str):
        hand_poses = []
        current_pose = self.move_group.get_current_pose().pose
        print("current pose: ", current_pose)
        with open(json_file, "r") as f:
            hand_poses = json.load(f)
        for i, hand_pose in enumerate(hand_poses):
            pose_goal = self._json_to_pose(hand_pose)
            # pose_goal = self.adjust_grasp_pose(pose_goal)
            self.display_pose(pose_goal)
            hand_config = self._json_to_hand_config(hand_pose)
            self.move_group.set_planner_id("RRTstar")
            self.move_group.set_pose_target(pose_goal)
            if self.get_ik(pose_goal).error_code.val != 1:
                rospy.loginfo("ik for pose #{} failed, trying next".format(i))
                continue
            try:
                print("planning pose #{}".format(i))
                result = self.move_group.plan()

                (success, traj, *_) = result

                if not success:
                    print("plan failed")
                    continue
                traj = self.move_group.retime_trajectory(
                    self.robot.get_current_state(),
                    traj,
                    velocity_scaling_factor=0.15,
                    acceleration_scaling_factor=0.15,
                    algorithm="time_optimal_trajectory_generation",
                )

                text = input(
                    "input anything to execute, press enter to skip, ctrl+d to exit"
                )
                if text:
                    traj_json = json_message_converter.convert_ros_message_to_json(traj)
                    json.dump(
                        traj_json,
                        open(
                            "/home/lab/hand_12/humanoidmanipulation/hand_arm_ws/src/leju_controller/hand_config/traj_py.json",
                            "w",
                        ),
                    )
                    hand_json = json_message_converter.convert_ros_message_to_json(
                        hand_config
                    )
                    json.dump(
                        hand_json,
                        open(
                            "/home/lab/hand_12/humanoidmanipulation/hand_arm_ws/src/leju_controller/hand_config/hand_py.json",
                            "w",
                        ),
                    )
                    # exit(0)
                    self.execute_plan(traj, hand_config)
                    return
            except EOFError:
                print("ctrl+d, exiting")
                return
            except KeyboardInterrupt:
                print("keyboard interrupted")
                return
            except Exception as e:
                print(e)

    def lift_hand(self):
        current_pose = self.move_group.get_current_pose().pose
        print("current pose: ", current_pose)
        current_pose.position.z += 0.2
        self.move_group.set_pose_target(current_pose)
        (success, traj, *_) = self.move_group.plan()
        # (success, traj, *_) = result

        if not success:
            print("plan failed!!!")
            
        traj = self.move_group.retime_trajectory(
            self.robot.get_current_state(),
            traj,
            velocity_scaling_factor=0.15,
            acceleration_scaling_factor=0.15,
            algorithm="time_optimal_trajectory_generation",
                )

        text = input(
            "input anything to execute, press enter to skip, ctrl+d to exit"
                )
        exit(0)
        # self.execute_arm_config(traj.joint_trajectory)

    def _json_to_pose(self, hand_pose: Any):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = hand_pose["r"]["x"]
        pose_goal.orientation.y = hand_pose["r"]["y"]
        pose_goal.orientation.z = hand_pose["r"]["z"]
        pose_goal.orientation.w = hand_pose["r"]["w"]
        pose_goal.position.x = hand_pose["t"]["x"]
        pose_goal.position.y = hand_pose["t"]["y"]
        pose_goal.position.z = hand_pose["t"]["z"]
        return pose_goal

    def _json_to_hand_config(self, hand_pose: Any, prefix="lh_"):
        state = JointState()
        joint_names = [k for k in hand_pose.keys() if k not in ["r", "t", "name"]]
        state.name = [prefix + k for k in joint_names]
        state.position = [hand_pose[k] for k in joint_names]
        return state

    def display_pose(self, pose):
        display_pose_publisher = rospy.Publisher(
            "/move_group/display_pose",
            geometry_msgs.msg.PoseStamped,
            queue_size=1,
        )
        for _ in range(10):
            pose_stamped = geometry_msgs.msg.PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = "world"
            pose_stamped.header.stamp = rospy.Time.now()
            display_pose_publisher.publish(pose_stamped)
            rospy.sleep(0.1)
        return

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_hand_config_sim(self, hand_config: JointState):
        if rospy.get_param("use_sim_time", False):
            inverse_joints = [
                "lh_thumb2",
                "lh_thumb3",
                "lh_index2",
                "lh_middle1",
                "lh_middle2",
                "lh_middle",
                "lh_ring1",
                "lh_ring2",
                "lh_ring3",
            ]
            for i, name in enumerate(hand_config.name):
                if name in inverse_joints:
                    hand_config.position[i] = -hand_config.position[i]
        print(hand_config)
        current_state = self.get_robot_joint_state()
        for i in range(1, 51):
            config_msg = copy.deepcopy(hand_config)

            for j, name in enumerate(hand_config.name):
                config_msg.position[j] = (
                    hand_config.position[j] - current_state[name]
                ) / 50 * i + current_state[name]

            self.joint_command_publisher.publish(config_msg)
            rospy.sleep(0.02)

    @staticmethod
    def interpolate_plan(plan: JointTrajectory, scale: int = 10):
        """
        插值优化
        
        :param plan: 待插值轨迹
        :param scale: 比例
        
        :return: 已插值轨迹
        """
        new_plan = JointTrajectory()
        new_plan.header = plan.header
        new_plan.joint_names = plan.joint_names
        new_plan.points = []
        for i in range(len(plan.points) - 1):
            point = plan.points[i]
            next_point = plan.points[i + 1]
            new_plan.points.append(point)
            t = 0
            while t < 1:
                new_point = JointTrajectoryPoint()
                new_point.time_from_start = (
                    point.time_from_start
                    + (next_point.time_from_start - point.time_from_start) * t
                )
                new_point.positions = []
                for j in range(len(point.positions)):
                    new_point.positions.append(
                        point.positions[j]
                        + (next_point.positions[j] - point.positions[j]) * t
                    )
                print("vel: ", point.velocities)
                new_point.velocities = point.velocities
                new_plan.points.append(new_point)
                t += 1 / scale
        new_plan.points.append(plan.points[-1])
        return new_plan

    def execute_arm_config(self, plan: JointTrajectory):
        """
        执行手臂控制
        
        :param plan: 待执行的轨迹点
        """
        print("executing arm config...")
        plan = self.interpolate_plan(plan)
        # print(plan)
        duration = rospy.Duration(0)
        for point in plan.points:
            # point = JointTrajectoryPoint()
            config_msg = JointState()
            config_msg.name = plan.joint_names
            config_msg.position = point.positions
            config_msg.velocity = point.velocities
            config_msg.header.stamp = rospy.Time.now()
            self.arm_command_publisher.publish(config_msg)
            # print(config_msg)
            rospy.sleep(point.time_from_start - duration + rospy.Duration(0, nsecs=3e4))
            duration = point.time_from_start

    def execute_hand_config(self, config: JointState):
        self.hand_config_publisher.publish(config)

    def execute_plan(self, plan: RobotTrajectory, hand_config: JointState):
        # self.move_group.execute(plan)
        self.execute_arm_config(plan.joint_trajectory)
        rospy.sleep(2)
        self.execute_hand_config(hand_config)
        # current_joints = move_group.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.01)
        # self.attach_cylinder()

    def add_bottle(self, pose=None):
        name = "bottle"
        """
        [-0.016163.0.23695,0.59945],[-0.5,0.5,-0.5,0.5]
        """
        if pose is None:
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = self.robot.get_planning_frame()
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = 0.36013184282525253
            pose.pose.position.y = 0.22643333155065256
            pose.pose.position.z = 0.8993937170152739

            pose.pose.orientation.x = -0.010915592726321414
            pose.pose.orientation.y = -0.007130583363980949
            pose.pose.orientation.z = -0.5490925999337581
            pose.pose.orientation.w = 0.8356598119535891
        height = 0.22
        radius = 0.075 / 2
        self.scene.add_cylinder(name, pose, height, radius)

    def add_drawer(self):
        name = "drawer"
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = 0.63
        p.pose.position.y = 0.4
        p.pose.position.z = 1.15 - 0.37 - 0.3
        self.scene.add_box(name, p, (1, 1.2, 0.6))

    def add_cylinder(self, timeout=4):
        name = "cylinder"
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.28
        pose.pose.position.y = 0.15
        pose.pose.position.z = 0.823
        self.scene.add_mesh(
            name,
            pose,
            "/home/lab/hand_12/humanoidmanipulation/hand_arm_ws/src/HD00_with_hand/meshes/cylinder_medium.stl",
        )
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the object is in the scene.
            # Note that attaching the object will remove it from known_objects
            is_known = name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if is_known == True:
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def attach_cylinder(self):
        self.scene.attach_mesh(
            self.eef_link, "cylinder", touch_links=self.touch_links_left
        )

    def detach_cylinder(self):
        self.scene.remove_attached_object(self.eef_link, "cylinder")

    def remove_cylinder(self):
        self.scene.remove_world_object("cylinder")

    def clear_environment(self):
        self.scene.remove_attached_object(self.eef_link)
        self.scene.remove_world_object()

    def get_robot_joint_state(self) -> Dict[str, float]:
        state = self.robot.get_current_state()
        joint_state = {}
        for i, name in enumerate(state.joint_state.name):
            joint_state[name] = state.joint_state.position[i]
        # print(joint_state)

        return joint_state

    def reset_fingers_sim(self):
        """
        重置手指位置
        """
        for _ in range(30):
            config_msg = JointState()
            config_msg.name = self.touch_links_left
            config_msg.position = [0.0] * len(config_msg.name)
            self.joint_command_publisher.publish(config_msg)
            rospy.sleep(0.02)


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin by setting up the moveit_commander ..."
        )
        controller = LejuController()
        # rospy.Subscriber("/joint_states", JointState, controller.record_arm_command)

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        # controller.home()

        input("============ Press `Enter` to execute a movement using a pose goal ...")

        controller.goto_joint_config(ARM_JOINT_CONFIG)
        # controller.test()
        rospy.sleep(5)
        # exit(0)
        controller.goto_hand_pose(
            "/home/lab/hand_12/humanoidmanipulation/hand_arm_ws/src/example/grasppose@contactdb+test.json"
        )
        controller.lift_hand()
        # controller.go_to_pose_goal()

        # input(
        #     "============ Press `Enter` to plan and display a Cartesian path ..."
        # )
        # cartesian_plan, fraction = controller.plan_cartesian_path()

        # input(
        #     "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # )
        # controller.display_trajectory(cartesian_plan)

        # input("============ Press `Enter` to execute a saved path ...")
        # controller.execute_plan(cartesian_plan)

        # input(
        #     "============ Press `Enter` to add a box to the planning scene ..."
        # )
        # controller.add_box()

        # input(
        #     "============ Press `Enter` to attach a Box to the Panda robot ..."
        # )
        # controller.attach_box()

        # input(
        #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # )
        # cartesian_plan, fraction = controller.plan_cartesian_path(scale=-1)
        # controller.execute_plan(cartesian_plan)

        # input(
        #     "============ Press `Enter` to detach the box from the Panda robot ..."
        # )
        # controller.detach_box()

        # input(
        #     "============ Press `Enter` to remove the box from the planning scene ..."
        # )
        # controller.remove_box()

        print("============ Python controller demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
