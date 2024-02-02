#!/usr/bin/env python3
import sys
sys.path.append("/home/lab/kuavo_moveit_ws/src/kuavo_arm_traj_planner/scripts")

import rospy
import sensor_msgs.msg

from utils import rad_to_angle, l_to_r, load_traj


def publish_arm_traj(publisher, traj) -> None:
    """发布左手轨迹
    """
    joint_state = sensor_msgs.msg.JointState()
    positions  = [0 for _ in range(14)]
    velocities = [0 for _ in range(14)]
    rate = rospy.Rate(3)
    
    for point in traj.joint_trajectory.points:
        if rospy.is_shutdown():
            rospy.logerr("用户终止程序")
            exit(0)
        positions[0:7] = rad_to_angle(point.positions)
        velocities[0:7] = point.velocities
        joint_state.position = positions
        joint_state.velocity = velocities
        
        publisher.publish(joint_state)
        rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("kuavo_arm_traj_plan", anonymous=True)

    publisher = rospy.Publisher(
        "/kuavo_arm_traj",
        sensor_msgs.msg.JointState,
        queue_size=1
    )
    
    traj = load_traj("config/traj_l_gua1.json")
    publish_arm_traj(publisher, traj)
    input("挂上灯笼=========:")
    traj = load_traj("config/traj_l_gua2.json")
    publish_arm_traj(publisher, traj)
    traj = load_traj("config/traj_l_gua3.json")
    publish_arm_traj(publisher, traj)
    input("等待运行=========:")
    traj = load_traj("config/traj_l_gua4.json")
    publish_arm_traj(publisher, traj)
    input("打开手抓=========:")
    traj = load_traj("config/traj_l_gua5.json")
    publish_arm_traj(publisher, traj)
    traj = load_traj("config/traj_l_gua6.json")
    publish_arm_traj(publisher, traj)
    traj = load_traj("config/traj_l_gua7.json")
    publish_arm_traj(publisher, traj)
    traj = load_traj("config/traj_l_gua8.json")
    publish_arm_traj(publisher, traj)


