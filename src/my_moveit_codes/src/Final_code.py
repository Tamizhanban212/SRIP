#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest, SetJointPosition, SetJointPositionRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tf_tr
import math
import matplotlib.pyplot as plt
import time
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import KinematicsPose
from kdl_parser_py.urdf import treeFromParam
import PyKDL
from scipy.signal import savgol_filter

# Global variables to store force readings, position readings, and timestamps
force_readings = []
position_readings = []
timestamps = []

def euler_to_quaternion(roll, pitch, yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
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

def detect_circles_and_transform():
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    readings = []

    for _ in range(10):
        ret, frame = cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=1000,
                                       param1=120, param2=40, minRadius=10, maxRadius=300)
            if circles is not None:
                circles = circles[0, :].round().astype("int")
                for (x, y, r) in circles:
                    x_new = y * 0.00039 + 0.048
                    y_new = (x - 335) * 0.00039
                    readings.append((x_new, y_new))
                    print(f"Transformed Coordinates: ({x_new:.2f}, {y_new:.2f})")
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
            cv2.imshow('Webcam', frame)
        if len(readings) >= 10 or cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

    if readings:
        x_new_mean = np.mean([x_new for x_new, y_new in readings])
        y_new_mean = np.mean([y_new for x_new, y_new in readings])
        return x_new_mean, y_new_mean
    else:
        return None, None

def get_kdl_chain():
    (ok, tree) = treeFromParam("/robot_description")
    if not ok:
        raise RuntimeError("Failed to parse the URDF file.")
    base_link = 'link1'
    end_link = 'end_effector_link'
    chain = tree.getChain(base_link, end_link)
    return chain

def get_jacobian(chain, joint_positions):
    jac_solver = PyKDL.ChainJntToJacSolver(chain)
    joint_array = PyKDL.JntArray(chain.getNrOfJoints())
    for i in range(chain.getNrOfJoints()):
        joint_array[i] = joint_positions[i]
    jacobian = PyKDL.Jacobian(chain.getNrOfJoints())
    jac_solver.JntToJac(joint_array, jacobian)
    J = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(jacobian.rows())])
    return J

def joint_state_callback(msg):
    joint_positions = msg.position
    joint_efforts = msg.effort
    torques = [(effort / 1000 - 0.0015) / 0.571428571 + 2.45 for effort in joint_efforts[:4]]
    T = np.array(torques).reshape(4, 1)
    chain = get_kdl_chain()
    J = get_jacobian(chain, joint_positions[:4])
    J_T = J.T
    U, sigma, Vt = np.linalg.svd(J_T, full_matrices=False)
    sigma_inv = np.diag([1.0 / s if s > 1e-6 else 0 for s in sigma])
    J_T_pseudo = Vt.T @ sigma_inv @ U.T
    F = J_T_pseudo @ T
    force_readings.append(F.flatten())
    timestamps.append(time.time())
    
def kinematics_pose_callback(msg):
    position = msg.pose.position
    position_readings.append({'x': position.x, 'y': position.y, 'z': position.z})

def send_pose(poses):
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
    start_time = time.time()
    for pose in poses:
        try:
            kinematics_pose = SetKinematicsPoseRequest()
            kinematics_pose.kinematics_pose.pose = pose
            kinematics_pose.planning_group = "arm"
            kinematics_pose.end_effector_name = "gripper"
            kinematics_pose.path_time = 2
            set_pose(kinematics_pose)
            rospy.sleep(2)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    end_time = time.time()
    rospy.sleep(0.01 * (len(timestamps) - (end_time - start_time) / 0.01))

def plot_forces_and_positions(apply_filter=False):
    start_time = timestamps[0]
    relative_time = [t - start_time for t in timestamps]
    Fx = [f[0] for f in force_readings]
    Fy = [f[1] for f in force_readings]
    Fz = [f[2] for f in force_readings]

    # Ensure that the number of position readings matches the number of force readings
    if len(position_readings) > len(force_readings):
        position_readings[:] = position_readings[:len(force_readings)]
    elif len(position_readings) < len(force_readings):
        position_readings.extend([position_readings[-1]] * (len(force_readings) - len(position_readings)))
    
    x_positions = [p['x'] for p in position_readings]
    y_positions = [p['y'] for p in position_readings]
    z_positions = [p['z'] for p in position_readings]

    if apply_filter:
        Fx = savgol_filter(Fx, 51, 3)  # window size 51, polynomial order 3
        Fy = savgol_filter(Fy, 51, 3)
        Fz = savgol_filter(Fz, 51, 3)
        x_positions = savgol_filter(x_positions, 51, 3)
        y_positions = savgol_filter(y_positions, 51, 3)
        z_positions = savgol_filter(z_positions, 51, 3)

    plt.figure(1)
    plt.plot(relative_time, Fx, label='Fx', color='r')
    plt.plot(relative_time, Fy, label='Fy', color='black')
    plt.plot(relative_time, Fz, label='Fz', color='b')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Force Components vs Time')
    plt.legend()

    plt.figure(2)
    plt.plot(relative_time, x_positions, label='x', color='r')
    plt.plot(relative_time, y_positions, label='y', color='black')
    plt.plot(relative_time, z_positions, label='z', color='b')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Position Components vs Time')
    plt.legend()

    plt.show()

if __name__ == "__main__":
    rospy.init_node('pose_gripper_control_node')
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.Subscriber("/gripper/kinematics_pose", KinematicsPose, kinematics_pose_callback)
    
    x_new_mean, y_new_mean = detect_circles_and_transform()
    if x_new_mean is not None and y_new_mean is not None:
        poses = [
            Pose(position=Point(x_new_mean, y_new_mean, 0.2), orientation=euler_to_quaternion(0, 0, 0)),
            Pose(position=Point(x_new_mean, y_new_mean, 0.13), orientation=euler_to_quaternion(0, 0, 0)),
            Pose(position=Point(x_new_mean, y_new_mean, 0.11), orientation=euler_to_quaternion(0, 0, 0)),
            Pose(position=Point(x_new_mean, y_new_mean, 0.2), orientation=euler_to_quaternion(0, 0, 0)),
            Pose(position=Point(0.05, 0.0, 0.25), orientation=euler_to_quaternion(0, 0, 0))
        ]
        send_pose(poses)
        rospy.sleep(0.01 * (len(timestamps) - (time.time() - timestamps[0]) / 0.01))
        plot_forces_and_positions(apply_filter=True)  # Set to True to apply the filter
    else:
        rospy.logerr("No circles detected or no readings obtained.")