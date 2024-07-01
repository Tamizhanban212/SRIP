#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
from trac_ik_python.trac_ik import IK
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam
import PyKDL

def get_kinematic_parameters():
    base_link = 'link1'
    end_link = 'end_effector_link'
    return base_link, end_link

def create_kdl_chain(base_link, end_link):
    (ok, tree) = treeFromParam("/robot_description")
    if not ok:
        raise RuntimeError("Failed to parse the URDF file.")
    chain = tree.getChain(base_link, end_link)
    return chain

def joint_state_callback(msg):
    joint_positions = msg.position
    joint_efforts = msg.effort

    # Convert efforts (current in milliamps) to torques
    torques = [(effort / 1000 - 0.0015) / 0.571428571 + 2.45 for effort in joint_efforts[:4]]  # Ignore gripper
    T = np.array(torques).reshape(4, 1)  # Torque matrix T

    # Get kinematic parameters
    base_link, end_link = get_kinematic_parameters()

    # Create KDL chain
    chain = create_kdl_chain(base_link, end_link)
    jac_solver = PyKDL.ChainJntToJacSolver(chain)

    # Create joint array
    joint_array = PyKDL.JntArray(chain.getNrOfJoints())
    for i in range(chain.getNrOfJoints()):
        joint_array[i] = joint_positions[i]

    # Compute Jacobian
    jacobian = PyKDL.Jacobian(chain.getNrOfJoints())
    jac_solver.JntToJac(joint_array, jacobian)

    # Convert PyKDL Jacobian to numpy array
    J = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(jacobian.rows())])

    # Compute pseudoinverse of the transpose of the Jacobian using SVD
    J_T = J.T  # Transpose of Jacobian
    U, sigma, Vt = np.linalg.svd(J_T, full_matrices=False)
    sigma_inv = np.diag([1.0 / s if s > 1e-6 else 0 for s in sigma])  # Handle singular values
    J_T_pseudo = Vt.T @ sigma_inv @ U.T

    # Compute end-effector force matrix F
    F = J_T_pseudo @ T

    # Log matrices
    rospy.loginfo("Joint Angles: %s", joint_positions[:4])  # Ignore gripper
    rospy.loginfo("Torque Matrix T:\n%s", T)
    rospy.loginfo("Jacobian Matrix J:\n%s", J)
    rospy.loginfo("End-Effector Force Matrix F:\n%s", F)

def listener():
    rospy.init_node('joint_state_listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass