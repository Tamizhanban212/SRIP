#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np

def joint_state_callback(msg):
    joint_names = msg.name
    joint_positions = msg.position
    joint_efforts = msg.effort

    # Convert efforts (current in milliamps) to torques
    torques = [(effort / 1000 - 0.0015) / 0.571428571 + 2.45 for effort in joint_efforts[:4]]  # Ignore gripper

    # Create the torque matrix T
    T = np.array(torques).reshape(4, 1)

    rospy.loginfo("Joint Angles: %s", joint_positions[:4])  # Ignore gripper
    rospy.loginfo("Torque Matrix T:\n%s", T)

def listener():
    rospy.init_node('joint_state_listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

