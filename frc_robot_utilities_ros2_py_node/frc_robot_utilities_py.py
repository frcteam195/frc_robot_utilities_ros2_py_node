#!/usr/bin/env python3

import tf2_ros
import rclpy
import rclpy.node
import rclpy.client
import nav_msgs.msg
import geometry_msgs.msg
from ck_ros2_base_msgs_node.msg import RobotStatus
from ck_ros2_msgs_node.msg import HMISignals
from frc_robot_utilities_ros2_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from frc_robot_utilities_ros2_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode
from robot_localization.srv import SetPose

from ck_utilities_ros2_py_node.node_handle import NodeHandle

from ck_utilities_ros2_py_node.geometry import *
from ck_utilities_ros2_py_node.ckmath import *

hmi_updates : BufferedROSMsgHandlerPy = None
robot_updates_internal : BufferedROSMsgHandlerPy = None
robot_status : RobotStatusHelperPy = None

set_pose : rclpy.client.Client = None

initialized = False

def register_for_robot_updates():
    global hmi_updates
    global robot_status
    global robot_updates_internal
    global set_pose
    global initialized

    if initialized: return

    hmi_updates = BufferedROSMsgHandlerPy(HMISignals)
    robot_updates_internal = BufferedROSMsgHandlerPy(RobotStatus)
    robot_status = RobotStatusHelperPy(robot_updates_internal)
    initialized = True

    node = NodeHandle().node_handle

    hmi_updates.register_for_updates("/HMISignals")
    robot_updates_internal.register_for_updates("/RobotStatus")

    set_pose = node.create_client(srv_type=SetPose, srv_name='/set_pose')

def reset_robot_pose(alliance : Alliance, x_inches=0, y_inches=0, heading_degrees=0):
    node = NodeHandle().node_handle

    odom = geometry_msgs.msg.PoseWithCovarianceStamped()
    odom.header.stamp = node.get_clock().now()
    odom.header.frame_id = 'odom'

    pose = Pose()
    pose.position.x = inches_to_meters(x_inches)
    pose.position.y = inches_to_meters(y_inches)
    pose.orientation.yaw = math.radians(heading_degrees)

    if alliance == Alliance.BLUE:
        pose.orientation.yaw += math.pi

    odom.pose.pose = pose.to_msg()

    pose_covariance = Covariance()
    pose_covariance.x_var = 0.01
    pose_covariance.y_var = 0.01
    pose_covariance.yaw_var = math.radians(1)

    odom.pose.covariance = pose_covariance.to_msg()

    set_pose_msg = SetPose.Request()
    set_pose_msg.pose = odom

    if (set_pose is not None):
        set_pose.call(set_pose_msg)
