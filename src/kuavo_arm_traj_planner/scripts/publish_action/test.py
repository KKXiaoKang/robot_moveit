#!/usr/bin/env python3
import sys
sys.path.append("/home/lab/kuavo_moveit_ws/src/kuavo_arm_traj_planner/scripts")

from core import Core
from scene import Scene
from interface import Interface
from utils import l_to_r

import math
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

if __name__ == "__main__":
    
    # scene = Scene()
    # scene.init_scene()
    
    # target_pose = geometry_msgs.msg.Pose()
    # target_pose.position.x = 0.36
    # target_pose.position.y = 0.41
    # target_pose.position.z = -0.1
    # support_pose = geometry_msgs.msg.Pose()
    # support_pose.position.x = 0.36
    # support_pose.position.y = 0.41
    # support_pose.position.z = -0.3
    # scene.add_target(target_pose)
    # scene.add_support(support_pose)
    
    # core = Core()
    # core.init_arm()
    
    # pose = geometry_msgs.msg.Pose()
    
    # pose.position.x = 0.16898946958744184
    # pose.position.x = 0.14898946958744184
    # pose.position.y = 0.40737878367102914
    # pose.position.z = 0.026838462099343076
    # pose.orientation.x = -0.2914717549806382
    # pose.orientation.y = -0.35449791421766536
    # pose.orientation.z = 0.5960406754158623
    # pose.orientation.w = 0.6588709722803353
    # traj = core.goto_pose_config(pose)
    
    # joints = [-20/180*math.pi, 0*math.pi, 0*math.pi, -70/180*math.pi, 0*math.pi, 0*math.pi, 0*math.pi]
    # traj = core.goto_joint_config(joints)
    
    # grasp = core.set_pick_config(pose)
    # traj = core.goto_pick_config("target", grasp)
    # place = core.set_place_config(pose)
    # traj = core.goto_place_config("target", place)
    
    # interface.publish_arm_traj(traj)
    
    
    core = Core()
    interface = Interface()
    core.init_arm()
    
    """当前坐标
    
    pose = core.get_l_current_pose()
    print(pose)
    
    """
    
    
    """拜年
    
    joints = [-60/180*math.pi, 0/180*math.pi, -35/180*math.pi, -60/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-20/180*math.pi, 0/180*math.pi, -35/180*math.pi, -60/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-60/180*math.pi, 0/180*math.pi, -35/180*math.pi, -60/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-20/180*math.pi, 0/180*math.pi, -35/180*math.pi, -60/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    
    """
    
    
    
    """拜年
    
    traj = core.goto_l_file_traj_config("config/traj_l_bainian1.json")
    traj = l_to_r(traj)
    core.r_move_group.execute(traj)
    traj = core.goto_l_file_traj_config("config/traj_l_bainian2.json")
    traj = l_to_r(traj)
    core.r_move_group.execute(traj)
    traj = core.goto_l_file_traj_config("config/traj_l_bainian3.json")
    traj = l_to_r(traj)
    core.r_move_group.execute(traj)
    traj = core.goto_l_file_traj_config("config/traj_l_bainian4.json")
    traj = l_to_r(traj)
    core.r_move_group.execute(traj)
    
    """
    
    
    """横幅
    
    joints = [-60/180*math.pi, -10/180*math.pi, -15/180*math.pi, -30/180*math.pi, 0/180*math.pi, -15/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-60/180*math.pi, 10/180*math.pi, 15/180*math.pi, -30/180*math.pi, 0/180*math.pi, 15/180*math.pi, 0/180*math.pi]
    traj = core.goto_r_joint_config(joints)
    joints = [-30/180*math.pi, 20/180*math.pi, -5/180*math.pi, -30/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-100/180*math.pi, -15/180*math.pi, 15/180*math.pi, -30/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_r_joint_config(joints)
    
    """
    
    
    
    
    """横幅
    
    traj = core.goto_l_file_traj_config("config/traj_l_hengfu1.json")
    traj = core.goto_r_file_traj_config("config/traj_r_hengfu1.json")
    traj = core.goto_l_file_traj_config("config/traj_l_hengfu2.json")
    traj = core.goto_r_file_traj_config("config/traj_r_hengfu2.json")
    
    """
    
    

    
    """灯笼1
    
    joints = [-55/180*math.pi, 0/180*math.pi, 0/180*math.pi, -45/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-70/180*math.pi, 0/180*math.pi, 0/180*math.pi, -30/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-30/180*math.pi, 0/180*math.pi, 0/180*math.pi, -70/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -85/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -50/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    
    """
    
    
    """灯笼1
    
    traj = core.goto_l_file_traj_config("config/traj_l_denglong1.json")
    traj = core.goto_l_file_traj_config("config/traj_l_denglong2.json")
    traj = core.goto_l_file_traj_config("config/traj_l_denglong3.json")
    traj = core.goto_l_file_traj_config("config/traj_l_denglong4.json")
    traj = core.goto_l_file_traj_config("config/traj_l_denglong5.json")
    
    """
    
    
    
    """灯笼2
    
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -50/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-70/180*math.pi, 70/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-145/180*math.pi, 50/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-140/180*math.pi, 40/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-140/180*math.pi, 70/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-140/180*math.pi, 95/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [-70/180*math.pi, 95/180*math.pi, -100/180*math.pi, -60/180*math.pi, 115/180*math.pi, -20/180*math.pi, 30/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, -50/180*math.pi, 0/180*math.pi, 0/180*math.pi, -10/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    
    """


    """

    traj = core.goto_l_file_traj_config("config/traj_l_gua1.json")
    traj = core.goto_l_file_traj_config("config/traj_l_gua2.json")
    traj = core.goto_l_file_traj_config("config/traj_l_gua3.json")
    traj = core.goto_l_file_traj_config("config/traj_l_gua4.json")
    traj = core.goto_l_file_traj_config("config/traj_l_gua5.json")
    traj = core.goto_l_file_traj_config("config/traj_l_gua6.json")
    traj = core.goto_l_file_traj_config("config/traj_l_gua7.json")
    traj = core.goto_l_file_traj_config("config/traj_l_gua8.json")
    
    """

    

    

