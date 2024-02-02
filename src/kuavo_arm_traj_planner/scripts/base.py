#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg

from utils import load_config

class Base(object):
    """moveit控制基类
    """
    
    def __init__(self) -> None:
        """moveit控制类初始化
        
        初始化所有控制器、执行器参数和发布结点
        
        :config
        :robot
        :scene
        :l_move_group
        :r_move_group
        :display_trajectory_publisher
        :kuavo_arm_traj_publisher
        
        :node_name
        :l_group_name
        :r_group_name
        :base_frame
        :l_planning_frame
        "r_planning_frame
        :l_eef_link
        :r_eef_link
        :group_names
        """
        if not rospy.core.is_initialized():
            rospy.init_node("kuavo_arm_traj_plan", anonymous=True)
            moveit_commander.roscpp_initialize(sys.argv)
        
        super(Base, self).__init__()
        
        self.config = load_config("config/kuavo.json")
        self.node_name = self.config["node_name"]
        self.l_group_name = self.config["l_group_name"]
        self.r_group_name = self.config["r_group_name"]
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.l_move_group = moveit_commander.MoveGroupCommander("l_arm_group")
        self.r_move_group = moveit_commander.MoveGroupCommander("r_arm_group")
        
        self.base_frame = self.robot.get_planning_frame()
        self.group_names = self.robot.get_group_names()
        self.l_planning_frame = self.l_move_group.get_planning_frame()
        self.r_planning_frame = self.r_move_group.get_planning_frame()
        self.l_eef_link = self.l_move_group.get_end_effector_link()
        self.r_eef_link = self.r_move_group.get_end_effector_link()
        
        self.init_move_group_config()
        self.init_publisher()
        
        
    def init_move_group_config(self):
        """初始化规划器配置
        """
        self.l_move_group.set_planner_id(self.config["planner_id"])
        self.l_move_group.set_pose_reference_frame(self.config["planning_frame"])
        self.l_move_group.set_num_planning_attempts(self.config["num_planning"])
        self.l_move_group.set_max_acceleration_scaling_factor(self.config["acceleration_scaling_factor"])
        self.l_move_group.set_max_velocity_scaling_factor(self.config["velocity_scaling_factor"])
        self.l_move_group.set_planning_time(self.config["planning_time"])
        self.l_move_group.set_support_surface_name(self.config["support"])
        
        self.r_move_group.set_planner_id(self.config["planner_id"])
        self.r_move_group.set_pose_reference_frame(self.config["planning_frame"])
        self.r_move_group.set_num_planning_attempts(self.config["num_planning"])
        self.r_move_group.set_max_acceleration_scaling_factor(self.config["acceleration_scaling_factor"])
        self.r_move_group.set_max_velocity_scaling_factor(self.config["velocity_scaling_factor"])
        self.r_move_group.set_planning_time(self.config["planning_time"])
        self.r_move_group.set_support_surface_name(self.config["support"])
    
    def init_publisher(self):
        """初始化发布器
        """
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
    