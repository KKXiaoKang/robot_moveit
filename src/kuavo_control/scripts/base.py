#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg

from utils import load_config

class Base(object):
    """moveit控制基类
    """
    
    __first_init = True
    def __init__(self) -> None:
        """moveit控制类初始化
        
        初始化所有控制器、执行器参数和发布结点
        
        :config
        :robot
        :scene
        :move_group
        :display_trajectory_publisher
        :kuavo_arm_traj_publisher
        
        :node_name
        :group_name
        :base_frame
        :planning_frame
        :eef_link
        :group_names
        """
        super(Base, self).__init__()
        
        if not self.__first_init:
            return
        else:
            self.__first_init = False
        
        self.config = load_config("config/kuavo.json")
        self.node_name = self.config["node_name"]
        self.group_name = self.config["group_name"]
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("kuavo_arm_group")
        
        self.move_group.set_planner_id(self.config["planner_id"])
        self.move_group.set_pose_reference_frame(self.config["planning_frame"])
        self.move_group.set_num_planning_attempts(self.config["num_planning"])
        self.move_group.set_max_acceleration_scaling_factor(self.config["acceleration_scaling_factor"])
        self.move_group.set_max_velocity_scaling_factor(self.config["velocity_scaling_factor"])
        self.move_group.set_planning_time(self.config["planning_time"])
        self.move_group.set_support_surface_name(self.config["support"])
        
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.kuavo_arm_traj_publisher = rospy.Publisher(
            "/kuavo_arm_traj",
            sensor_msgs.msg.JointState,
            queue_size=1
        )
        
        self.base_frame = self.robot.get_planning_frame()
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        
        
        