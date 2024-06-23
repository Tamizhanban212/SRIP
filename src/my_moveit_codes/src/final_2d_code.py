#!/usr/bin/env python
import cv2
import numpy as np
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

def detect_circles_and_transform():
    # Open webcam with specified resolution
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    readings = []

    # Take 10 readings
    for _ in range(10):
        # Capture frame-by-frame
        ret, frame = cap.read()

        if ret:
            # Convert frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Apply Hough Circle Transform to detect circles
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=1000,
                                       param1=120, param2=40, minRadius=10, maxRadius=300)

            if circles is not None:
                circles = circles[0, :].round().astype("int")

                for (x, y, r) in circles:
                    # Apply coordinate transformation
                    x_new = y * 0.00039 + 0.048
                    y_new = (x - 335) * 0.00039
                    readings.append((x_new, y_new))

                    # Print the transformed coordinates
                    print(f"Transformed Coordinates: ({x_new:.2f}, {y_new:.2f})")

                    # Draw the circle detected
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    # Draw the center of the circle
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

            # Display the frame
            cv2.imshow('Webcam', frame)

        # Exit the loop if 'q' is pressed or readings are enough
        if len(readings) >= 10 or cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and destroy any OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

    # Compute mean of x_new and y_new readings
    if readings:
        x_new_mean = np.mean([x_new for x_new, y_new in readings])
        y_new_mean = np.mean([y_new for x_new, y_new in readings])
        return x_new_mean, y_new_mean
    else:
        return None, None

def send_pose(poses):
    rospy.init_node('pose_gripper_control_node')
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)

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

if __name__ == "__main__":
    # Detect circles and compute mean of transformed coordinates
    x_new_mean, y_new_mean = detect_circles_and_transform()

    if x_new_mean is not None and y_new_mean is not None:
        # Define the poses to be executed sequentially
        poses = [
            Pose(position=Point(x_new_mean, y_new_mean, 0.2), orientation=euler_to_quaternion(0, 0, 0)),
            Pose(position=Point(x_new_mean, y_new_mean, 0.13), orientation=euler_to_quaternion(0, 0, 0)),
            Pose(position=Point(x_new_mean, y_new_mean, 0.11), orientation=euler_to_quaternion(0, 0, 0)),
            Pose(position=Point(x_new_mean, y_new_mean, 0.2), orientation=euler_to_quaternion(0, 0, 0)),
            Pose(position=Point(0.05, 0.0, 0.25), orientation=euler_to_quaternion(0, 0, 0))
        ]
        # Send the computed poses
        send_pose(poses)
    else:
        rospy.logerr("No circles detected or no readings obtained.")




# #!/usr/bin/env python
# import cv2
# import numpy as np
# import rospy
# from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest, SetJointPosition, SetJointPositionRequest
# from geometry_msgs.msg import Pose, Point, Quaternion
# import tf.transformations as tf_tr
# import math

# def euler_to_quaternion(roll, pitch, yaw):
#     # Convert degrees to radians
#     roll = math.radians(roll)
#     pitch = math.radians(pitch)
#     yaw = math.radians(yaw)
#     # Convert to quaternion
#     return Quaternion(*tf_tr.quaternion_from_euler(roll, pitch, yaw))

# def control_gripper(position):
#     rospy.wait_for_service('/goal_tool_control')
#     try:
#         tool_control = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
#         request = SetJointPositionRequest()
#         request.joint_position.joint_name = ["gripper"]
#         request.joint_position.position = [position]
#         tool_control(request)
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s", e)

# def detect_circles_and_transform():
#     # Open webcam with specified resolution
#     cap = cv2.VideoCapture(0)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
#     readings = []

#     # Take 10 readings
#     for _ in range(10):
#         # Capture frame-by-frame
#         ret, frame = cap.read()

#         if ret:
#             # Convert frame to grayscale
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#             # Apply Hough Circle Transform to detect circles
#             circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=1000,
#                                        param1=120, param2=40, minRadius=10, maxRadius=300)

#             if circles is not None:
#                 circles = circles[0, :].round().astype("int")

#                 for (x, y, r) in circles:
#                     # Apply coordinate transformation
#                     x_new = y * 0.00039 + 0.052
#                     y_new = (x - 320) * 0.00039
#                     readings.append((x_new, y_new))

#                     # Print the transformed coordinates
#                     print(f"Transformed Coordinates: ({x_new:.2f}, {y_new:.2f})")

#                     # Draw the circle detected
#                     cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
#                     # Draw the center of the circle
#                     cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

#             # Display the frame
#             cv2.imshow('Webcam', frame)

#         # Exit the loop if 'q' is pressed or readings are enough
#         if len(readings) >= 10 or cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     # Release the capture and destroy any OpenCV windows
#     cap.release()
#     cv2.destroyAllWindows()

#     # Compute mean of x_new and y_new readings
#     if readings:
#         x_new_mean = np.mean([x_new for x_new, y_new in readings])
#         y_new_mean = np.mean([y_new for x_new, y_new in readings])
#         return x_new_mean, y_new_mean
#     else:
#         return None, None

# def send_pose(poses):
#     rospy.init_node('pose_gripper_control_node')
#     rospy.wait_for_service('/goal_task_space_path')
#     set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)

#     for pose in poses:
#         try:
#             kinematics_pose = SetKinematicsPoseRequest()
#             kinematics_pose.kinematics_pose.pose = pose
#             kinematics_pose.planning_group = "arm"
#             kinematics_pose.end_effector_name = "gripper"
#             kinematics_pose.path_time = 2
#             set_pose(kinematics_pose)
#         except rospy.ServiceException as e:
#             rospy.logerr("Service call failed: %s", e)

#         rospy.sleep(2)

# if __name__ == "__main__":
#     # Detect circles and compute mean of transformed coordinates
#     x_new_mean, y_new_mean = detect_circles_and_transform()

#     if x_new_mean is not None and y_new_mean is not None:
#         # Define the poses to be executed sequentially
#         poses = [
#             Pose(position=Point(x_new_mean, y_new_mean, 0.2), orientation=euler_to_quaternion(0, 0, 0)),
#             Pose(position=Point(0.05, 0.0, 0.25), orientation=euler_to_quaternion(0, 0, 0))
#         ]
#         # Send the computed poses
#         send_pose(poses)
#     else:
#         rospy.logerr("No circles detected or no readings obtained.")

