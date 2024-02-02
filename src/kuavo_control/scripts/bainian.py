#!/usr/bin/env python3

from utils import load_traj, l_to_r, rad_to_angle

import rospy
import sensor_msgs.msg

def publish_lr_arm_traj(traj, publisher):
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
        
        publisher.publish(joint_state)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("kuavo_arm_traj_plan", anonymous=True)
    
    kuavo_arm_traj_publisher = rospy.Publisher(
        "/kuavo_arm_traj",
        sensor_msgs.msg.JointState,
        queue_size=1
    )
    
    """拜年动作 
    
    
    
    """
    
    traj = load_traj("config/traj_l_bainian1.json")
    publish_lr_arm_traj(traj, kuavo_arm_traj_publisher)
    traj = load_traj("config/traj_l_bainian2.json")
    publish_lr_arm_traj(traj, kuavo_arm_traj_publisher)
    traj = load_traj("config/traj_l_bainian3.json")
    publish_lr_arm_traj(traj, kuavo_arm_traj_publisher)
    traj = load_traj("config/traj_l_bainian4.json")
    publish_lr_arm_traj(traj, kuavo_arm_traj_publisher)

    