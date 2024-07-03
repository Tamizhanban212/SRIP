#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest, SetJointPosition, SetJointPositionRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tf_tr
import math

def euler_to_quaternion(roll, pitch, yaw):
    # Convert degrees to radians
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    # Convert to quaternion
    return Quaternion(*tf_tr.quaternion_from_euler(roll, pitch, yaw))

def control_gripper(position):
    rospy.wait_for_service('/goal_tool_control')
    try:
        tool_control = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = [position]
        tool_control(request)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def compute_orientation(x, y, z):
    # If z coordinate is below or equal to 0.1 meters, return (0,0,0) orientation
    if z <= 0.1:
        return (0, 0, 0)
    
    # Compute orientation based on x and y coordinates
    # Example heuristic: orientation towards the position from a reference point (e.g., (0,0,0))
    # Adjust this part based on specific requirements or heuristics
    roll = 0
    pitch = -math.atan2(y, z)  # Simple heuristic example
    yaw = math.atan2(x, z)     # Simple heuristic example
    
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))  # Return degrees for consistency

def send_pose(poses):
    rospy.init_node('pose_gripper_control_node')
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)

    for position, gripper_state, delay in poses:
        x, y, z = position
        orientation = compute_orientation(x, y, z)
        pose = Pose(position=Point(x, y, z), orientation=euler_to_quaternion(*orientation))
        
        try:
            kinematics_pose = SetKinematicsPoseRequest()
            kinematics_pose.kinematics_pose.pose = pose
            kinematics_pose.planning_group = "arm"
            kinematics_pose.end_effector_name = "gripper"
            kinematics_pose.path_time = 2
            set_pose(kinematics_pose)
            rospy.sleep(delay)
            if gripper_state != "unchanged":
                control_gripper(0.01 if gripper_state == "open" else -0.01)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        rospy.sleep(2)

if __name__ == "__main__":
    rospy.sleep(2)
    poses = [
        ((0.05, 0.0, 0.25), "close", 0.2), # Above 0.1 meters, should compute orientation
    ]
    send_pose(poses)