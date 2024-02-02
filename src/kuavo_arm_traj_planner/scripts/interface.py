#!/usr/bin/env python3
import math
import std_msgs.msg
import sensor_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg
import rospy

from base import Base
from core import Core
from utils import rad_to_angle, calibration_roban

class Interface(Base):
    """发布轨迹与接收目标位置接口类
    """
    def __init__(self) -> None:
        """发布轨迹接口类初始化
        """
        super(Interface, self).__init__()
        self.core = Core()
    
    def sub_camera(self):
        rospy.Subscriber(
            "/target_pose",
            geometry_msgs.msg.Pose,
            self.subscribe_camera_callback,
            queue_size=3
        )
        rospy.spin()
        
    def sub_roban(self):
        rospy.Subscriber(
            "/array_topic",
            std_msgs.msg.Float32MultiArray,
            self.subscribe_roban_callback,
            queue_size=3
        )
        rospy.spin()
    
    
    def publish_arm_traj(self, traj: moveit_msgs.msg.RobotTrajectory) -> None:
        """发布左手轨迹
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
    
    def publish_lr_arm_traj(self, l_traj, r_traj) -> None:
        """发布左右手轨迹
        """
        joint_state = sensor_msgs.msg.JointState()
        positions  = [0 for _ in range(14)]
        velocities = [0 for _ in range(14)]
        positions_temp = [0 for _ in range(7)]
        rate = rospy.Rate(3)
        
        if len(l_traj.joint_trajectory.points) > len(r_traj.joint_trajectory.points):
            for i in range(len(r_traj.joint_trajectory.points)):
                if rospy.is_shutdown():
                    rospy.logerr("用户终止程序")
                    exit(0)
                positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
                positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
                velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
                velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
                joint_state.position = positions
                joint_state.velocity = velocities
                
                positions_temp = r_traj.joint_trajectory.points[i].positions
                self.kuavo_arm_traj_publisher.publish(joint_state)
                rate.sleep()
                
            for i in range(len(r_traj.joint_trajectory.points) , len(l_traj.joint_trajectory.points)):
                if rospy.is_shutdown():
                    rospy.logerr("用户终止程序")
                    exit(0)
                positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
                positions[7:14] = rad_to_angle(positions_temp)
                velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
                velocities[7:14] = [0 for _ in range(7)]
                joint_state.position = positions
                joint_state.velocity = velocities
                
                self.kuavo_arm_traj_publisher.publish(joint_state)
                rate.sleep()
        else:
            for i in range(len(l_traj.joint_trajectory.points)):
                if rospy.is_shutdown():
                    rospy.logerr("用户终止程序")
                    exit(0)
                positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
                positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
                velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
                velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
                joint_state.position = positions
                joint_state.velocity = velocities
                
                positions_temp = l_traj.joint_trajectory.points[i].positions
                self.kuavo_arm_traj_publisher.publish(joint_state)
                rate.sleep()
            
            for i in range(len(l_traj.joint_trajectory.points) , len(r_traj.joint_trajectory.points)):
                if rospy.is_shutdown():
                    rospy.logerr("用户终止程序")
                    exit(0)
                positions[0:7] = rad_to_angle(positions_temp)
                positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
                velocities[0:7] = [0 for _ in range(7)]
                velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
                joint_state.position = positions
                joint_state.velocity = velocities
                
                self.kuavo_arm_traj_publisher.publish(joint_state)
                rate.sleep()
    
    def subscribe_camera_callback(self, pose: geometry_msgs.msg.Pose) -> None:
        """回调函数
        """
        
        traj = self.core.goto_l_pose_config(pose)
        self.publish_arm_traj(traj)
        
    def subscribe_roban_callback(self, joints: std_msgs.msg.Float32MultiArray) -> None:
        """回调函数
        """
        joints = list(joints.data)
        joints = calibration_roban(joints)
        rospy.loginfo(joints)
        self.l_move_group.go(joints)
        

    
    
    