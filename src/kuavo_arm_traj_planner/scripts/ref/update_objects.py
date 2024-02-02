import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from ros_numpy import numpify, msgify


def get_bottle_pose(adjust_pose=None):
    bottle_pose: TransformStamped = rospy.wait_for_message(
        "/vicon/bottle/bottle", TransformStamped, rospy.Duration(1)
    )
    arm_pose: TransformStamped = rospy.wait_for_message(
        "/vicon/leju_shoulder/leju_shoulder", TransformStamped, rospy.Duration(1)
    )
    if not bottle_pose or not arm_pose:
        raise RuntimeError("no bottle pose or arm pose found from vicon")

    bottle_transformation = numpify(bottle_pose.transform)
    arm_transformation = numpify(arm_pose.transform)
    arm_to_bottle = np.linalg.inv(arm_transformation) @ bottle_transformation
    if adjust_pose is not None:
        arm_to_bottle = arm_to_bottle @ adjust_pose
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = bottle_pose.header.frame_id
    pose_stamped.pose = msgify(Pose, arm_to_bottle)
    return pose_stamped


def main():
    rospy.init_node("update_objects")
