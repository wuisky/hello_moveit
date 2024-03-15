"""
Sample script to run Moveit clients
"""
import copy
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from hello_moveit.msg import CollisionPair
from hello_moveit.srv import (ApplyCollisionObject,
                              ApplyCollisionObjectFromMesh, AttachHand,
                              CheckCollision, DetachHand,
                              PlanExecuteCartesianPath, PlanExecutePoses)
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Byte, Int32


def apply_collision_object(
        node: Node,
        operation: Byte = CollisionObject.ADD) -> bool:
    """
    ROS service client for applying object to the planning scene

    Parameters
    ----------
    node: Node
        Node which will receive server's responces
    operation: Byte, default CollisionObject.ADD
        Operation flag which you want to apply

    Returns
    -------
    ret_code: bool
        Result of calling ROS service
    """
    apply_collision_object_cli = node.create_client(ApplyCollisionObject,
                                                    'apply_collision_object')
    obj = CollisionObject()
    obj.id = 'wall'
    obj.operation = operation
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.BOX
    sp.dimensions = [0.0] * 3
    sp.dimensions[SolidPrimitive.BOX_X] = 0.1
    sp.dimensions[SolidPrimitive.BOX_Y] = 1.0
    sp.dimensions[SolidPrimitive.BOX_Z] = 1.0
    obj.primitives.append(sp)

    obj_pose = Pose()
    obj_pose.orientation.w = 1.0
    obj_pose.orientation.x = 0.0
    obj_pose.orientation.y = 0.0
    obj_pose.orientation.z = 0.0
    obj_pose.position.x = 0.3
    obj_pose.position.y = 0.0
    obj_pose.position.z = 0.5
    # obj_pose.orientation.w = 0.5
    # obj_pose.orientation.x = 0.5
    # obj_pose.orientation.y = -0.5
    # obj_pose.orientation.z = 0.5
    # obj_pose.position.x = 1.3
    # obj_pose.position.y = 0.85
    # obj_pose.position.z = 0.1
    obj.primitive_poses.append(obj_pose)
    req = ApplyCollisionObject.Request()
    req.object = obj
    while not apply_collision_object_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('apply_collison_object service not ready, sleep 1sec')
    future = apply_collision_object_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ret_code = future.result().is_success
    if ret_code is not True:
        node.get_logger().error('apply object fail!!')
    else:
        node.get_logger().info('success!')
    return ret_code

def apply_collision_object_from_mesh(
        node: Node,
        operation: Byte = CollisionObject.ADD) -> bool:
    """
    ROS service client for applying object which is loaded from mesh file to the planning scene

    Parameters
    ----------
    node: Node
        Node which will receive server's responces
    operation: Byte, default CollisionObject.ADD
        Operation flag which you want to apply

    Returns
    -------
    ret_code: bool
        Result of calling ROS service
    """
    apply_collision_object_from_mesh_cli = node.create_client(ApplyCollisionObjectFromMesh,
                                                              'apply_collision_object_from_mesh')
    req = ApplyCollisionObjectFromMesh.Request()
    req.resource_path = 'package://hello_moveit/cad/shelf_rev5/type1_slid_shelf_change_1.STL'
    req.object_id = 'shelf'
    req.scale = 0.001
    obj_pose = Pose()
    obj_pose.orientation.w = 1.0
    obj_pose.orientation.x = 0.0
    obj_pose.orientation.y = 0.0
    obj_pose.orientation.z = 0.0
    obj_pose.position.x = -0.70932
    obj_pose.position.y = 0.43848
    obj_pose.position.z = -0.83605
    req.pose = obj_pose
    req.operation = operation

    while not apply_collision_object_from_mesh_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('appply_collision_object_from_mesh service not ready, sleep 1sec')
    future = apply_collision_object_from_mesh_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ret_code = future.result().is_success
    if ret_code is not True:
        node.get_logger().error('apply object from mesh fail!!')
    else:
        node.get_logger().info('success!')
    return ret_code

def attach_hand(node: Node) -> bool:
    """
    ROS service client for applying hand to tcp of the robot

    Parameters
    ----------
    node: Node
        Node which will receive server's responces

    Returns
    -------
    ret_code: bool
        Result of calling ROS service
    """
    attach_hand_cli = node.create_client(AttachHand, 'attach_hand')
    req = AttachHand.Request()
    req.resource_path = ('package://hello_moveit/cad/robotiq_gripper/'
                         'robotiq_2F_adaptive_gripper_rough.STL')
    req.object_id = 'robotiq_hand'
    req.scale = 0.001
    grab_pose = Pose()
    grab_pose.orientation.w = 1.0
    grab_pose.position.z = 0.0
    req.pose = grab_pose
    req.touch_links = ['wrist_3_link']

    while not attach_hand_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('attach hand service not ready, sleep 1sec')
    future = attach_hand_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ret_code = future.result().is_success
    if ret_code is not True:
        node.get_logger().error('attach hand fail!!')
    else:
        node.get_logger().info('success!')
    return ret_code

def check_collision(node: Node) -> List[CollisionPair]:
    """
    ROS service client for checking the collision between objects
    
    Parameters
    ----------
    node: Node
        Node which will receive server's responces

    Returns
    -------
    collision_pairs: List[CollisionPair]
        List of names which are expected to collide each other
    """
    check_collision_cli = node.create_client(CheckCollision, 'check_collision')
    req = CheckCollision.Request()
    js = JointState()
    js.name = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
        'wrist_2_joint', 'wrist_3_joint'
    ]
    # check if collision happen when all joint angel is 0.0
    js.position = [0.0] * len(js.name)
    node.get_logger().info(f'pos: {js.position}')
    req.joint_state = js
    while not check_collision_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('check_collision service not ready, sleep 1sec')
    future = check_collision_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    collision_pairs = future.result().collision_pairs
    if collision_pairs:
        node.get_logger().warn('\n'.join(f'{pairs}' for pairs in collision_pairs))
    else:
        node.get_logger().info('no collision')
    return collision_pairs

def detach_hand(node: Node) -> bool:
    """
    ROS service client for detaching hand of the robot

    Parameters
    ----------
    node: Node
        Node which will receive server's responces

    Returns
    -------
    ret_code: bool
        Result of calling ROS service
    """
    detach_hand_cli = node.create_client(DetachHand, 'detach_hand')
    req = DetachHand.Request()
    req.object_id = 'robotiq_hand'

    while not detach_hand_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('detach hand service not ready, sleep 1sec')
    future = detach_hand_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ret_code = future.result().is_success
    if ret_code is not True:
        node.get_logger().error('detach hand fail!!')
    else:
        node.get_logger().info('success!')
    return ret_code

def plan_execute_poses(
        node: Node,
        pose: Pose) -> Int32:
    """
    ROS service client for planning to achieve the target pose of tcp

    Parameters
    ----------
    node: Node
        Node which will receive server's responces
    pose: Pose
        Target tcp pose

    Returns
    -------
    ret_code: Int32
        Result of calling ROS service
    """
    plan_execute_poses_cli = node.create_client(PlanExecutePoses, 'plan_execute_poses')
    req = PlanExecutePoses.Request()
    req.velocity_scale = 1.0  # you can chage this to slow down robot
    req.poses = [pose]

    while not plan_execute_poses_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')

    future = plan_execute_poses_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ret_code = future.result().err_code.val
    if ret_code is not MoveItErrorCodes.SUCCESS:
        node.get_logger().error(f'fail!!MoveItErrorCode: {future.result().err_code.val}')
    else:
        node.get_logger().info('success!')
    return ret_code

def plan_execute_cartesian_path(
        node: Node,
        poses: List[Pose]) -> bool:
    """
    ROS service client for planning to execute the cartesian path of tcp

    Parameters
    ----------
    node: Node
        Node which will receive server's responces
    pose: Pose
        List of target tcp poses

    Returns
    -------
    ret_code: bool
        Result of calling ROS service
    """
    plan_execute_cartesian_cli = node.create_client(PlanExecuteCartesianPath,
                                                    'plan_execute_cartesian_path')
    req = PlanExecuteCartesianPath.Request()
    req.velocity_scale = 1.0  # you can chage this to slow down robot
    req.poses = poses

    while not plan_execute_cartesian_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')

    future = plan_execute_cartesian_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ret_code = future.result().is_success
    if not ret_code:
        node.get_logger().error('faill cartesian_path')
    else:
        node.get_logger().info('success!')
    return ret_code

def compute_fk(
    node: Node,
    joint_state: Optional[JointState] = None) -> Tuple[Pose, Int32]:
    """
    ROS service client for computing the forward kinematics

    Parameters
    ----------
    node: Node
        Node which will receive server's responces
    joint_state: JointState
        Message which contains the joint states and names of the robot
        If the inputted joint state is empty, FK is computed by using the current joint_state

    Returns
    -------
    tcp_pose: Pose
        Tcp pose corresponding to the joint state
    ret_code: Int32
        Result of calling ROS service
    """
    # create service client (type, service name)
    compute_fk_cli = node.create_client(GetPositionFK, 'compute_fk')
    # initialize and substitute request
    req = GetPositionFK.Request()
    req.header.frame_id = 'base_link'
    req.fk_link_names = ['wrist_3_link']
    if joint_state:
        req.robot_state.joint_state = joint_state
    # wait connection to server
    while not compute_fk_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')
    # send requests and get responses
    future = compute_fk_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future) # wait until getting response
    rsp = future.result()
    ret_code = rsp.error_code.val
    # check response
    if ret_code is not MoveItErrorCodes.SUCCESS:
        node.get_logger().error('fail to compute FK')
    else:
        node.get_logger().info('success!')
    # show response
    tcp_pose = rsp.pose_stamped[0].pose
    return tcp_pose, ret_code

def compute_ik(
    node: Node,
    target_tcp_pose: Pose,
    initial_joint_state: Optional[JointState] = None) -> Tuple[JointState, Int32]:
    """
    ROS service client for computing the inverse kinematics

    Parameters
    ----------
    node: Node
        Node which will receive server's responces
    target_tcp_pose: Pose
        Desired tcp pose
    initial_joint_state: Optional[JointState], default None
        Initial joint state of the robot which become a hint of computing the IK
        If the inputted joint state is empty, IK is computed by using the current joint_state

    Returns
    -------
    target_joint_state: JointState
        Desired joint states to achieve the given tcp pose
    ret_code: Int32
        Result of calling ROS service
    """
    # create service client (type, service name)
    compute_ik_cli = node.create_client(GetPositionIK, 'compute_ik')
    # initialize and substitute request
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'ur_manipulator'
    if initial_joint_state:
        req.ik_request.robot_state.joint_state = initial_joint_state
    req.ik_request.avoid_collisions = True
    req.ik_request.pose_stamped = PoseStamped()
    req.ik_request.pose_stamped.pose = target_tcp_pose
    # wait connection to server
    while not compute_ik_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')
    # send requests and get responses
    future = compute_ik_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future) # wait until getting response
    rsp = future.result()
    ret_code = rsp.error_code.val
    # check response
    if ret_code is not MoveItErrorCodes.SUCCESS:
        node.get_logger().error('fail to compute IK')
    else:
        node.get_logger().info('success!')
    # show response
    target_joint_state = rsp.solution.joint_state
    return target_joint_state, ret_code

def main() -> None:
    """
    Sample sequence of Moveit clients
    """
    rclpy.init()
    node = rclpy.create_node('oreore')

    apply_collision_object(node)
    apply_collision_object_from_mesh(node)
    attach_hand(node)
    check_collision(node)
    # set target pose1
    msg = Pose()
    msg.orientation.w = -0.5
    msg.orientation.x = 0.5
    msg.orientation.y = 0.5
    msg.orientation.z = -0.5
    msg.position.x = -0.696
    msg.position.y = 0.052
    msg.position.z = 0.464
    node.get_logger().info(f'target tcp pose : {msg}')

    # get target joint state for moving from current pose to the target pose1
    target_joint_state = compute_ik(node, target_tcp_pose=msg)
    node.get_logger().info(f'target joint state : {target_joint_state}')

    # send target pose1
    plan_execute_poses(node, msg)

    # get current pose of tcp
    current_tcp_pose = compute_fk(node)
    node.get_logger().info(f'current tcp pose : {current_tcp_pose}')

    # send target pose2
    msg = Pose()
    msg.orientation.w = -0.5
    msg.orientation.x = 0.5
    msg.orientation.y = 0.5
    msg.orientation.z = -0.5
    msg.position.x = -0.696
    msg.position.y = 0.0519
    msg.position.z = 0.154
    plan_execute_poses(node, msg)
    # send catesian path
    waypoint1 = copy.deepcopy(msg)
    waypoint2 = copy.deepcopy(msg)
    waypoint1.position.y += 0.2
    plan_execute_cartesian_path(node, [waypoint1, waypoint2])
    # remove object
    apply_collision_object_from_mesh(node, CollisionObject.REMOVE)
    apply_collision_object(node, CollisionObject.REMOVE)
    detach_hand(node)

    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
