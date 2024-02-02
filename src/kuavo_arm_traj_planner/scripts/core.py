#!/usr/bin/env python3

import rospy
import trajectory_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg

from base import Base
from optimizer import Optimizer
from utils import load_joints, load_traj, dump_traj


class Core(Base):
    """规划核心
    """

    def __init__(self) -> None:
        """规划核心类初始化
        """
        super(Core, self).__init__()
        self.optimizer = Optimizer()
    
    def init_arm(self) -> None:
        """初始化手臂位置
        """
        joints = [0 for _ in range(7)]
        self.l_move_group.go(joints, wait=True)
        self.r_move_group.go(joints, wait=True)
    
    def goto_l_joint_config(self, joints: list) -> moveit_msgs.msg.RobotTrajectory:
        """前往左手关节目标位置
        
        最近一次轨迹保存在config/traj_l.json中
        
        :param joints: 目标位置
        :return: 已优化轨迹
        """
        self.l_move_group.set_joint_value_target(joints)
        
        trajs = list()
        for i in range(self.config["num_planning"]):
            rospy.loginfo("第{}次关节空间规划".format(i+1))
            (success, traj, *_) = self.l_move_group.plan()
            if success:
                rospy.loginfo("第{}次规划成功".format(i+1))
                trajs.append(traj)
            else:
                rospy.logwarn("第{}次规划失败".format(i+1))

        if len(trajs) == 0:
            rospy.logerr("规划失败")
            exit(0)

        traj = self.optimizer.min_variance_traj(trajs)
        traj = self.optimizer.retime_l_traj(traj)
        dump_traj("config/traj_l.json", traj)
        self.l_move_group.execute(traj)

        return traj
    
    def goto_r_joint_config(self, joints: list) -> moveit_msgs.msg.RobotTrajectory:
        """前往右手关节目标位置
        
        最近一次轨迹保存在config/traj_r.json中
        
        :param joints: 目标位置
        :return: 已优化轨迹
        """
        self.r_move_group.set_joint_value_target(joints)
        
        trajs = list()
        for i in range(self.config["num_planning"]):
            rospy.loginfo("第{}次关节空间规划".format(i+1))
            (success, traj, *_) = self.r_move_group.plan()
            if success:
                rospy.loginfo("第{}次规划成功".format(i+1))
                trajs.append(traj)
            else:
                rospy.logwarn("第{}次规划失败".format(i+1))

        if len(trajs) == 0:
            rospy.logerr("规划失败")
            exit(0)

        traj = self.optimizer.min_variance_traj(trajs)
        traj = self.optimizer.retime_r_traj(traj)
        dump_traj("config/traj_r.json", traj)
        self.r_move_group.execute(traj)

        return traj
    
    def goto_l_pose_config(self, pose: geometry_msgs.msg.Pose) -> moveit_msgs.msg.RobotTrajectory:
        """前往左手笛卡尔目标位置
        
        最近一次轨迹保存在config/traj_l.json中
        
        :param joints: 目标位置
        :return: 已优化轨迹
        """
        self.l_move_group.set_pose_target(pose)
        
        trajs = list()
        for i in range(self.config["num_planning"]):
            rospy.loginfo("第{}次笛卡尔空间规划".format(i+1))
            (success, traj, *_) = self.l_move_group.plan()
            if success:
                rospy.loginfo("第{}次规划成功".format(i+1))
                trajs.append(traj)
            else:
                rospy.logwarn("第{}次规划失败".format(i+1))

        if len(trajs) == 0:
            rospy.logerr("规划失败")
            exit(0)

        traj = self.optimizer.min_variance_traj(trajs)
        traj = self.optimizer.retime_l_traj(traj)
        dump_traj("config/traj_l.json", traj)
        self.l_move_group.execute(traj)

        return traj
    
    def goto_r_pose_config(self, pose: geometry_msgs.msg.Pose) -> moveit_msgs.msg.RobotTrajectory:
        """前往笛卡尔目标位置
        
        最近一次轨迹保存在config/traj_r.json中
        
        :param joints: 目标位置
        :return: 已优化轨迹
        """
        self.r_move_group.set_pose_target(pose)
        
        trajs = list()
        for i in range(self.config["num_planning"]):
            rospy.loginfo("第{}次笛卡尔空间规划".format(i+1))
            (success, traj, *_) = self.r_move_group.plan()
            if success:
                rospy.loginfo("第{}次规划成功".format(i+1))
                trajs.append(traj)
            else:
                rospy.logwarn("第{}次规划失败".format(i+1))

        if len(trajs) == 0:
            rospy.logerr("规划失败")
            exit(0)

        traj = self.optimizer.min_variance_traj(trajs)
        traj = self.optimizer.retime_r_traj(traj)
        dump_traj("config/traj_r.json", traj)
        self.r_move_group.execute(traj)

        return traj

    def set_pick_config(self, pose: geometry_msgs.msg.Pose) -> moveit_msgs.msg.Grasp:
        """设置抓取配置
        
        :param pose: 抓取位姿
        :return: 抓取配置
        """
        
        grasp = moveit_msgs.msg.Grasp()
        grasp.grasp_pose.header.frame_id = self.l_planning_frame
        grasp.grasp_pose.header.stamp = rospy.Time.now()
        
        grasp.grasp_pose.pose = pose
        
        grasp.pre_grasp_approach.direction.vector.x = 1
        grasp.pre_grasp_approach.min_distance = 0.04
        grasp.pre_grasp_approach.desired_distance = 0.05
        
        # grasp.post_grasp_retreat.direction.vector = ...
        # grasp.post_grasp_retreat.min_distance = ...
        # grasp.post_grasp_retreat.desired_distance = ...
        
        self.open_gripper_config(grasp.pre_grasp_posture)
        self.close_gripper_config(grasp.grasp_posture)
        
        return grasp
        
        
    def set_place_config(self, pose: geometry_msgs.msg.Pose) -> moveit_msgs.msg.PlaceLocation:
        """设置放置配置
        
        :param place: 放置位姿
        :return: 放置配置
        """
        
        place = moveit_msgs.msg.PlaceLocation()
        place.place_pose.header.frame_id = self.l_planning_frame
        place.place_pose.header.stamp = rospy.Time.now()
        
        place.place_pose.pose = pose
        
        # place.pre_place_approach.direction.vector = ...
        # place.pre_place_approach.min_distance = ...
        # place.pre_place_approach.desired_distance = ...
        
        # place.post_place_retreat.direction.vector.x = -1
        # place.post_place_retreat.min_distance = 0.02
        # place.post_place_retreat.desired_distance = 0.04
        
        self.open_gripper_config(place.post_place_posture)
        
        return place
    
    def goto_pick_config(self, target: str, grasp: moveit_msgs.msg.Grasp) -> moveit_msgs.msg.RobotTrajectory:
        """前往抓取配置
        
        :param target: 抓取目标
        :param grasp: 抓取位姿
        :return: 抓取轨迹
        """
        trajs = list()
        for i in range(self.config["num_planning"]):
            rospy.loginfo("第{}次笛卡尔空间抓取".format(i+1))
            result = self.l_move_group.pick(target, grasp)
            if result == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                try:
                    traj_msg = rospy.wait_for_message(
                        "/move_group/display_planned_path",
                        moveit_msgs.msg.DisplayTrajectory,
                        timeout=3)
                    trajs = traj_msg.trajectory
                    rospy.loginfo("抓取成功")
                    break
                except rospy.ROSException as e:
                    rospy.logerr("未收到轨迹消息：{}".format(str(e)))
                    exit(0)
        else:
            rospy.logerr("抓取失败")
            exit(0)
        
        traj = self.optimizer.min_variance_traj(trajs)
        traj = self.optimizer.retime_l_traj(traj)
        dump_traj("config/traj_l.json", traj)
        
        return traj
        
    def goto_place_config(self, target: str, place: moveit_msgs.msg.PlaceLocation) -> moveit_msgs.msg.RobotTrajectory:
        """前往放置配置
        
        :param target: 放置目标
        :param grasp: 放置位姿
        :return: 放置轨迹
        """
        trajs = list()
        for i in range(self.config["num_planning"]):
            rospy.loginfo("第{}次笛卡尔空间放置".format(i+1))
            result = self.l_move_group.place(target, place)
            if result == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                try:
                    traj_msg = rospy.wait_for_message(
                        "/move_group/display_planned_path",
                        moveit_msgs.msg.DisplayTrajectory,
                        timeout=3)
                    trajs = traj_msg.trajectory
                    rospy.loginfo("放置成功")
                    break
                except rospy.ROSException as e:
                    rospy.logerr("未收到轨迹消息：{}".format(str(e)))
                    exit(0)
        else:
            rospy.logerr("放置失败")
            exit(0)

        traj = self.optimizer.min_variance_traj(trajs)
        traj = self.optimizer.retime_l_traj(traj)
        dump_traj("config/traj_l.json", traj)
        
        return traj
    
    def open_gripper_config(self, posture: trajectory_msgs.msg.JointTrajectory) -> None:
        """打开手指配置

        :param posture: 打开手指位置
        """
        posture.joint_names = self.config["l_gripper"]
        open_gripper = trajectory_msgs.msg.JointTrajectoryPoint()
        open_gripper.positions = [0.02, 0.02]
        open_gripper.time_from_start = rospy.Duration(1)
        posture.points.append(open_gripper)
    
    def close_gripper_config(self, posture: trajectory_msgs.msg.JointTrajectory) -> None:
        """关闭手指配置

        :param target_pose: 关闭手指位置
        """
        posture.joint_names = self.config["r_gripper"]
        close_gripper = trajectory_msgs.msg.JointTrajectoryPoint()
        close_gripper.positions = [0, 0]
        close_gripper.time_from_start = rospy.Duration(1)
        posture.points.append(close_gripper)
        
    def goto_l_file_traj_config(self, path: str) -> moveit_msgs.msg.RobotTrajectory:
        """执行文件左手固定轨迹
        """
        traj = load_traj(path)
        self.l_move_group.execute(traj)
        
        return traj
    
    def goto_r_file_traj_config(self, path: str) -> moveit_msgs.msg.RobotTrajectory:
        """执行文件右手固定轨迹
        """
        traj = load_traj(path)
        self.r_move_group.execute(traj)
        
        return traj
        
    def goto_l_file_joint_config(self, path: str) -> moveit_msgs.msg.RobotTrajectory:
        """执行文件左手固定轨迹点
        """
        joints = load_joints(path)
        traj = moveit_msgs.msg.RobotTrajectory()
        traj.joint_trajectory.header.frame_id = self.l_planning_frame
        traj.joint_trajectory.header.stamp = rospy.Time.now()
        for joint in joints:
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = joint
            traj.joint_trajectory.points.append(point)

        dump_traj("config/traj_l.json", traj)
        self.l_move_group.execute(traj)
        
        return traj
    
    def goto_r_file_joint_config(self, path: str) -> moveit_msgs.msg.RobotTrajectory:
        """执行文件右手固定轨迹点
        """
        joints = load_joints(path)
        traj = moveit_msgs.msg.RobotTrajectory()
        traj.joint_trajectory.header.frame_id = self.r_planning_frame
        traj.joint_trajectory.header.stamp = rospy.Time.now()
        for joint in joints:
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = joint
            traj.joint_trajectory.points.append(point)

        dump_traj("config/traj_l.json", traj)
        self.r_move_group.execute(traj)
        
        return traj
        
    def get_l_current_pose(self) -> geometry_msgs.msg.PoseStamped:
        """获取当前末端执行器笛卡儿位姿
        """
        return self.l_move_group.get_current_pose()
    
    def get_r_current_pose(self) -> geometry_msgs.msg.PoseStamped:
        """获取当前末端执行器笛卡儿位姿
        """
        return self.r_move_group.get_current_pose()
    
    def get_l_current_joints(self) -> list:
        """获取当前关节角度信息
        """
        return self.l_move_group.get_joints()
    
    def get_r_current_joints(self) -> list:
        """获取当前关节角度信息
        """
        return self.r_move_group.get_joints()
    
        