#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

from interface import Interface
from scene import Scene


if __name__ == "__main__":
    rospy.init_node("kuavo_arm_traj_plan", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    
    scene = Scene()
    scene.init_scene()
    
    intreface = Interface()
    intreface.start_subscribe()
