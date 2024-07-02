#!/usr/bin/env python

import rospy
from open_manipulator_msgs.msg import KinematicsPose

def kinematics_pose_callback(msg):
    # Handle the received KinematicsPose message
    position = msg.pose.position
    print(f"Received position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}")

if __name__ == "__main__":
    rospy.init_node("kinematics_pose_subscriber")
    rospy.Subscriber("/gripper/kinematics_pose", KinematicsPose, kinematics_pose_callback)
    rospy.spin()
