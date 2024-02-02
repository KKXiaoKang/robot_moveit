#!/usr/bin/env python3

import trajectory_msgs.msg
import moveit_msgs.msg
import typing
import numpy as np
import rospy

from base import Base

class Optimizer(Base):
    """轨迹优化类
    """
    
    def __init__(self) -> None:
        """初始化优化类
        """
        super(Optimizer, self).__init__()
        
    def retime_traj(self, traj: moveit_msgs.msg.RobotTrajectory) -> moveit_msgs.msg.RobotTrajectory:
        """重新计时轨迹优化
        
        :param traj: 待优化轨迹
        :return: 优化轨迹
        """
        traj = self.move_group.retime_trajectory(
            self.robot.get_current_state(),
            traj,
            velocity_scaling_factor=0.5,
            acceleration_scaling_factor=0.5,
            algorithm="time_optimal_trajectory_generation",
        )
        
        return traj
        
    def min_variance_traj(self, trajs: typing.List[moveit_msgs.msg.RobotTrajectory]) -> moveit_msgs.msg.RobotTrajectory:
        """计算最小方差轨迹
        
        :param trajs: 轨迹数组
        :return: 具有最小方差的轨迹
        """
        if len(trajs) == 1:
            rospy.loginfo("选中唯一轨迹")
            return trajs[0]
        
        variances = [0 for _ in range(len(trajs))]
        position_matrix = list()
        for i, traj in enumerate(trajs):
            points = traj.joint_trajectory.points
            for point in points:
                position_matrix.append(point.positions)
            position_matrix_T = np.array(position_matrix).T
            for positions in position_matrix_T:
                variances[i] += np.var(positions)
            rospy.loginfo("第{}条轨迹方差：{}".format(i+1, variances[i]))
        
        rospy.loginfo("选中第{}条轨迹".format(variances.index(min(variances))+1))
        return trajs[variances.index(min(variances))]
            
    def interpolate_traj(traj: trajectory_msgs.msg.JointTrajectory, scale: int = 10) -> trajectory_msgs.msg.JointTrajectory:
        """轨迹插值优化
        
        :param traj: 待插值轨迹
        :param scale: 比例
        :return: 已插值轨迹
        """
        new_traj = trajectory_msgs.msg.JointTrajectory()
        new_traj.header = traj.header
        new_traj.joint_names = traj.joint_names
        new_traj.points = []
        for i in range(len(traj.points) - 1):
            point = traj.points[i]
            next_point = traj.points[i + 1]
            new_traj.points.append(point)
            t = 0
            while t < 1:
                new_point = trajectory_msgs.JointTrajectoryPoint()
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
                rospy.loginfo("vel: {}".format(point.velocities))
                new_point.velocities = point.velocities
                new_traj.points.append(new_point)
                t += 1 / scale
        new_traj.points.append(traj.points[-1])
        return new_traj
    
    