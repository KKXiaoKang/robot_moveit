#!/usr/bin/env python3
import sys
sys.path.append("/home/lab/kuavo_moveit_ws/src/kuavo_arm_traj_planner/scripts")

import rospy
import sensor_msgs.msg

from utils import rad_to_angle, l_to_r, load_traj


def publish_lr_arm_traj(publisher, l_traj, r_traj) -> None:
    """发布轨迹
    """
    joint_state = sensor_msgs.msg.JointState()
    positions  = [0 for _ in range(14)]
    velocities = [0 for _ in range(14)]
    rate = rospy.Rate(3)
    
    if len(l_traj.joint_trajectory.points) > len(r_traj.joint_trajectory.points):
        for i in range(len(r_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
            
        for i in range(len(r_traj.joint_trajectory.points) , len(l_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = [0 for _ in range(7)]
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = [0 for _ in range(7)]
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
        
    else:
        for i in range(len(l_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = l_traj.joint_trajectory.points[i].velocities
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
            joint_state.position = positions
            joint_state.velocity = velocities
            
            publisher.publish(joint_state)
            rate.sleep()
        for i in range(len(l_traj.joint_trajectory.points) , len(r_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = [0 for _ in range(7)]
            positions[7:14] = rad_to_angle(r_traj.joint_trajectory.points[i].positions)
            velocities[0:7] = [0 for _ in range(7)]
            velocities[7:14] = r_traj.joint_trajectory.points[i].velocities
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
    
    l_traj = load_traj("config/traj_l_bainian1.json")
    r_traj = l_to_r(l_traj)
    publish_lr_arm_traj(publisher, l_traj, r_traj)
    l_traj = load_traj("config/traj_l_bainian2.json")
    r_traj = l_to_r(l_traj)
    publish_lr_arm_traj(publisher, l_traj, r_traj)
    l_traj = load_traj("config/traj_l_bainian3.json")
    r_traj = l_to_r(l_traj)
    publish_lr_arm_traj(publisher, l_traj, r_traj)
    l_traj = load_traj("config/traj_l_bainian4.json")
    r_traj = l_to_r(l_traj)
    publish_lr_arm_traj(publisher, l_traj, r_traj)
