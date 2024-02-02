#!/usr/bin/env python3

import sensor_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg
import rospy

from base import Base
from core import Core
from utils import rad_to_angle, l_to_r

class Interface(Base):
    """发布轨迹与接收目标位置接口类
    """
    def __init__(self) -> None:
        """发布轨迹接口类初始化
        """
        super(Interface, self).__init__()
        self.core = Core()
    
    def start_subscribe(self):
        rospy.Subscriber(
            "/target_pose",
            geometry_msgs.msg.Pose,
            self.subscribe_callback,
            queue_size=3
        )
        rospy.spin()
    
    def publish_arm_traj(self, traj: moveit_msgs.msg.RobotTrajectory) -> None:
        """发布轨迹
        """
        joint_state = sensor_msgs.msg.JointState()
        positions  = [0 for _ in range(14)]
        velocities = [0 for _ in range(14)]
        rate = rospy.Rate(1)
        
        for point in traj.joint_trajectory.points:
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(point.positions)
            velocities[0:7] = point.velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            self.kuavo_arm_traj_publisher.publish(joint_state)
            rate.sleep()
            
    def publish_lr_arm_traj(self, traj: moveit_msgs.msg.RobotTrajectory) -> None:
        """发布轨迹
        """
        joint_state = sensor_msgs.msg.JointState()
        positions  = [0 for _ in range(14)]
        velocities = [0 for _ in range(14)]
        rate = rospy.Rate(1)
        
        for point in traj.joint_trajectory.points:
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(point.positions)
            positions[7:14] = l_to_r(positions[0:7])
            velocities[0:7] = point.velocities
            velocities[7:14] = point.velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            self.kuavo_arm_traj_publisher.publish(joint_state)
            rate.sleep()
    
    def subscribe_callback(self, pose: geometry_msgs.msg.Pose) -> None:
        """回调函数
        """
        traj = self.core.goto_pose_config(pose)
        self.publish_arm_traj(traj)
    
    
    
    