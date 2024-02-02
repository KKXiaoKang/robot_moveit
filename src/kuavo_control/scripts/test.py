#!/usr/bin/env python3

from core import Core
from scene import Scene
from interface import Interface

import math
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

if __name__ == "__main__":
    rospy.init_node("kuavo_arm_traj_plan", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    
    scene = Scene()
    scene.init_scene()
    
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
    
    core = Core()
    interface = Interface()
    core.init_arm()
    
    # planning_frame = base.move_group.get_planning_frame()
    # print(" ===== planning frame %s" % planning_frame)
    # eef_link = base.move_group.get_end_effector_link()
    # print(" ===== end effector %s" % eef_link)
    # print(" ===== robot state")
    # print(base.robot.get_current_state())
    
    # pose = geometry_msgs.msg.Pose()
    
    # pose.position.x = 0.16898946958744184
    # pose.position.x = 0.14898946958744184
    # pose.position.y = 0.40737878367102914
    # pose.position.z = 0.026838462099343076
    # pose.orientation.x = -0.2914717549806382
    # pose.orientation.y = -0.35449791421766536
    # pose.orientation.z = 0.5960406754158623
    # pose.orientation.w = 0.6588709722803353
    
    # joints = [-50/180*math.pi, 33/180*math.pi, 120/180*math.pi, 80/180*math.pi, 10/180*math.pi, -12/180*math.pi, 15/180*math.pi]
    # traj = core.goto_joint_config(joints)
    
    # joints = [-12/180*math.pi, 33/180*math.pi, 120/180*math.pi, 80/180*math.pi, 10/180*math.pi, -12/180*math.pi, 15/180*math.pi]
    # traj = core.goto_joint_config(joints)
    
    # grasp = core.set_pick_config(pose)
    # traj = core.goto_pick_config("target", grasp)
    
    # traj = core.goto_pose_config(pose)
    
    traj = core.goto_file_traj_config("config/traj_l_bainian1.json")
    interface.publish_lr_arm_traj(traj)
    traj = core.goto_file_traj_config("config/traj_l_bainian2.json")
    interface.publish_lr_arm_traj(traj)
    traj = core.goto_file_traj_config("config/traj_l_bainian3.json")
    interface.publish_lr_arm_traj(traj)
    traj = core.goto_file_traj_config("config/traj_l_bainian4.json")
    interface.publish_lr_arm_traj(traj)
    
    # interface.publish_arm_traj(traj)
    
    # # place = core.set_place_config(pose)
    # # traj = core.goto_place_config("target", place)
    # pose.position.x = 0.25
    # pose.position.y = 0.45
    # pose.position.z = 0.1
    # traj = core.goto_pose_config(pose)
    # interface.publish_arm_traj(traj)
    
    # pose = core.get_current_pose()
    # print(pose)
    