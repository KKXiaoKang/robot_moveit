#!/usr/bin/env python3
import sys
sys.path.append("/home/lab/kuavo_moveit_ws/src/kuavo_arm_traj_planner/scripts")

import rospy
import sensor_msgs.msg

from utils import rad_to_angle, load_traj


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

def publish_lr_arm_traj(publisher, l_traj, r_traj) -> None:
    """发布轨迹
    """
    joint_state = sensor_msgs.msg.JointState()
    positions  = [0 for _ in range(14)]
    velocities = [0 for _ in range(14)]
    positions_temp = [0 for _ in range(7)]
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
            
            positions_temp = r_traj.joint_trajectory.points[i].positions
            publisher.publish(joint_state)
            rate.sleep()
        
        for i in range(len(r_traj.joint_trajectory.points) , len(l_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(l_traj.joint_trajectory.points[i].positions)
            positions[7:14] = rad_to_angle(positions_temp)
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
            
            positions_temp = l_traj.joint_trajectory.points[i].positions
            publisher.publish(joint_state)
            rate.sleep()
        for i in range(len(l_traj.joint_trajectory.points) , len(r_traj.joint_trajectory.points)):
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            positions[0:7] = rad_to_angle(positions_temp)
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

    """功能一：发布左手动作
    
    traj = load_traj("traj/...")
    publish_arm_traj(publisher, traj)

    """

    traj = load_traj("traj/traj_l_zhuahua1")
    publish_arm_traj(publisher, traj)
    traj = load_traj("traj/traj_l_zhuahua2")
    publish_arm_traj(publisher, traj)
    traj = load_traj("traj/traj_l_zhuahua3")
    publish_arm_traj(publisher, traj)
    traj = load_traj("traj/traj_l_zhuahua4")
    publish_arm_traj(publisher, traj)
    traj = load_traj("traj/traj_l_zhuahua5")
    publish_arm_traj(publisher, traj)
    
    """功能二：发布左右手动作
    
    l_traj = load_traj("traj/...")
    r_traj = load_traj("traj/...")
    publish_lr_arm_traj(publisher, l_traj, r_traj)
    
    """

