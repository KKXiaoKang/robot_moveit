#!/usr/bin/env python3
import sys
sys.path.append("/home/lab/kuavo_moveit_ws/src/kuavo_arm_traj_planner/scripts")

from core import Core
from utils import dump_traj

import math

traj_file_name = "traj/traj_l_zhuahua"

KeyFrames_degrees = [
    [-20, 0, 0, 0, 0, 0,  0],
    [-40, 0, 0, 0, 0, 0,  0],
    [-50, 0, 0, 0, 0, 0,  0],
    [-55, 0, 0, 0, 0, 0,  0],
    [-100, 0, 0, 0, 0, 0, 0]
]

def degrees_to_radians(degrees):
    return [angle/180 * math.pi for angle in degrees]

def generate_and_dump_traj(core, joints, filename_template):
    traj = core.goto_l_joint_config(joints)
    filename = f"{filename_template}.json"
    dump_traj(filename, traj)

if __name__ == "__main__":
    core = Core()
    core.init_arm()

    # 设计动作
    for i, degrees in enumerate(KeyFrames_degrees, start=1):
        radians = degrees_to_radians(degrees)
        generate_and_dump_traj(core, radians, f"{traj_file_name}_{i}")

    # # 查看设计模板
    # for i in range(5):
    #     traj = core.goto_l_file_traj_config(f"traj/traj_l_zhuahua_{i+1}.json")
