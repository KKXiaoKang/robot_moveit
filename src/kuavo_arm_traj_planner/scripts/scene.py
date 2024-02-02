#!/usr/bin/env python3

import geometry_msgs.msg
import rospy

from base import Base

class Scene(Base):
    """场景更新类
    """
    
    def __init__(self) -> None:
        """场景更新类初始化
        """
        super(Scene, self).__init__()

    def init_scene(self) -> None:
        """初始化场景
        """
        self.scene.remove_attached_object(self.l_eef_link)
        self.scene.remove_world_object()
        
    def add_target(self, target_pose: geometry_msgs.msg.Pose) -> None:
        """添加目标
        """
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.l_planning_frame
        pose.header.stamp = rospy.Time.now()

        pose.pose = target_pose
        
        height = 0.2
        radius = 0.01
        self.scene.add_cylinder("target", pose, height, radius)
        
    def add_support(self, support_pose: geometry_msgs.msg.Pose) -> None:
        """添加支撑物
        """
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.l_planning_frame
        pose.header.stamp = rospy.Time.now()
        
        pose.pose = support_pose
        
        size = (0.3, 0.6, 0.2)
        self.scene.add_box("support", pose, size)
    
    
    