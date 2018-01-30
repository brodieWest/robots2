#!/usr/bin/env python

# ROS Python API
import rospy

import tf2_ros
from transforms3d import quaternions
from transforms3d.derivations import eulerangles
import numpy as np


# Baxter SDK that provides high-level functions to control Baxter
import baxter_interface
from baxter_interface import CHECK_VERSION



#The Header message comprises a timestamp, message sequence id and frame id where the message originated from (see http://docs.ros.org/api/std_msgs/html/msg/Header.html)
from std_msgs.msg import Header

# Pose is the defacto message to store a 3D point and a quaternion, head over ROS API to find out the structure!
# PoseStamped consists of a Header and a Pose messages. 
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

# Baxter ROS messages and services that allows communicating with the robot. The Inverse Kinematic function runs inside the robot, and it is closed-source. To access it, the developers have made available the SolvePositionIK service; hence we need to import it, and also, its "request" message (see: https://github.com/RethinkRobotics/baxter_common/blob/master/baxter_core_msgs/srv/SolvePositionIK.srv)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

arm = "left"
pose_point = [0.644, 0.0, 0.066]
pose_orientation = [-0.381, 0.923, -0.015, 0.052]

def ik_baxter():
    rospy.init_node("rsdk_ik_service_client")

    # Connect to Baxter's service (see: http://sdk.rethinkrobotics.com/wiki/API_Reference#Inverse_Kinematics_Solver_Service)
    srv_name = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
    rospy.wait_for_service(srv_name)
    srv = rospy.ServiceProxy(srv_name, SolvePositionIK)

    # Setup Baxter SDK
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rospy.loginfo("Enabling Baxter")
    # You need to enable Baxter to use it!
    rs.enable()

    # Move to home position
    move_neutral_position()

    # Indicate the SDK which arm we are going to use
    armcmd = baxter_interface.Limb(arm)

    # Generate the Pose message to be passed to the IK service
    pose_arm = generate_posemsg(pose_point, pose_orientation)

    # Call the Inverse Kinematic service
    limb_joints = ik_service(srv, pose_arm)

    # Move arm
    if limb_joints is not None:
        armcmd.move_to_joint_positions(limb_joints)

    # After moving to the desired location, tell the SDK that you stopped using the arm, this will disable the arm at the same time
    armcmd.exit_control_mode()

    return 0

def move_neutral_position():
    _left_arm = baxter_interface.Limb("left")
    _right_arm = baxter_interface.Limb("right")
    _left_arm.move_to_neutral()
    _right_arm.move_to_neutral()
    _left_arm.exit_control_mode()
    _right_arm.exit_control_mode()

def generate_posemsg(point, orientation):
    # Building up a PoseStamped message request
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    pt = Point(x=point[0], y=point[1], z=point[2])
    qt = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
    ps = Pose(position = pt, orientation = qt)
    pose_arm = PoseStamped(header=hdr, pose=ps)

    return pose_arm


def ik_service(srv, pose_arm):
    request = SolvePositionIKRequest()
    request.pose_stamp.append(pose_arm)
    try:
        resp = srv(request)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return None

    # Format solution into Limb API-compatible dictionary
    limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    print limb_joints
    return limb_joints

def query_transformation(from_frame, to_frame):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trans = None
    quat = None
    try:
        transformation = tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time(), rospy.Duration(1.0))
        trans = transformation.transform.translation
        quat = transformation.transform.rotation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Transformation is not available")

    return trans, quat

def rotate_pose(point, quat, theta, axis="x"):

    # set quaternion to transforms3d format, i.e. w,x,y,z
    quat_t3d = [quat[3], quat[0], quat[1], quat[2]]
    rot_mat = quaternions.quat2mat(quat_t3d)

    H1 = np.eye(4)
    H1[:3,:3] = rot_mat
    H1[:3,3] = point

    H2 = np.eye(4)
    if axis == "x":
        rot_axis = np.array(eulerangles.x_rotation(theta))
    elif axis == "y":
        rot_axis = np.array(eulerangles.y_rotation(theta))
    elif axis == "z":
        rot_axis = np.array(eulerangles.z_rotation(theta))

    print rot_axis
    H2[:3,:3] = rot_axis

    H3 = np.dot(H1, H2)
    point_out = H3[:3,3]
    quat_t3dout = quaternions.mat2quat(H3[:3,:3])
    # Turn to ROS format, i.e. x, y, z, w
    quat_out = [quat_t3dout[1], quat_t3dout[2], quat_t3dout[3], quat_t3dout[0]]
    rospy.loginfo("Rotated pose: ")
    print point_out, quat_out

    return point_out, quat_out




if __name__ == '__main__':
    import sys
    sys.exit(ik_baxter())

