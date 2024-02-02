#!/usr/bin/env python3
import sys
sys.path.append("/home/lab/kuavo_moveit_ws/src/kuavo_arm_traj_planner/scripts")

from core import Core
from utils import dump_traj

import math

if __name__ == "__main__":
    core = Core()
    core.init_arm()


    """设计动作模板
    
    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    dump_traj("traj/...", traj)
    traj = core.goto_r_joint_config(joints)
    dump_traj("traj/...", traj)

    joints = [0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    dump_traj("traj/...", traj)
    traj = core.goto_r_joint_config(joints)
    dump_traj("traj/...", traj)
    
    """

    """
    
    joints = [-50/180*math.pi, 40/180*math.pi, 80/180*math.pi, 20/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    dump_traj("traj/traj_l_zhuahua1.json", traj)

    joints = [-80/180*math.pi, 35/180*math.pi, 90/180*math.pi, 50/180*math.pi, 0/180*math.pi, 0/180*math.pi, 40/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    dump_traj("traj/traj_l_zhuahua2.json", traj)

    joints = [-80/180*math.pi, 25/180*math.pi, 90/180*math.pi, 50/180*math.pi, 0/180*math.pi, 0/180*math.pi, 40/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    dump_traj("traj/traj_l_zhuahua3.json", traj)

    joints = [-95/180*math.pi, 25/180*math.pi, 90/180*math.pi, 50/180*math.pi, 0/180*math.pi, 0/180*math.pi, 40/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    dump_traj("traj/traj_l_zhuahua4.json", traj)

    joints = [-95/180*math.pi, 10/180*math.pi, 90/180*math.pi, 20/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi]
    traj = core.goto_l_joint_config(joints)
    dump_traj("traj/traj_l_zhuahua5.json", traj)
    
    """

    """查看设计模板

    traj = core.goto_l_file_traj_config("traj/traj_l_zhuahua1.json")
    traj = core.goto_l_file_traj_config("traj/traj_l_zhuahua2.json")
    traj = core.goto_l_file_traj_config("traj/traj_l_zhuahua3.json")
    traj = core.goto_l_file_traj_config("traj/traj_l_zhuahua4.json")
    traj = core.goto_l_file_traj_config("traj/traj_l_zhuahua5.json")
    
    """
    traj = core.goto_l_file_traj_config("../traj/traj_l_zhuahua1.json")
    traj = core.goto_l_file_traj_config("../traj/traj_l_zhuahua2.json")
    traj = core.goto_l_file_traj_config("../traj/traj_l_zhuahua3.json")
    traj = core.goto_l_file_traj_config("../traj/traj_l_zhuahua4.json")
    traj = core.goto_l_file_traj_config("../traj/traj_l_zhuahua5.json")
    

