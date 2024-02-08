import copy

from geometry_msgs.msg import Pose
from hello_moveit.srv import (ApplyCollisionObject,
                              ApplyCollisionObjectFromMesh, AttachHand,
                              CheckCollision, DetachHand,
                              PlanExecuteCartesianPath, PlanExecutePoses)
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
import rclpy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


def apply_collision_object(node, operation=CollisionObject.ADD):
    apply_collision_object_cli = node.create_client(ApplyCollisionObject, 'apply_collision_object')
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
    if future.result().is_success is not True:
        node.get_logger().error('apply object fail!!')
    else:
        node.get_logger().info('success!')

def apply_collision_object_from_mesh(node, operation=CollisionObject.ADD):
    apply_collision_object_from_mesh_cli = node.create_client(ApplyCollisionObjectFromMesh, 'apply_collision_object_from_mesh')
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
    if future.result().is_success is not True:
        node.get_logger().error('apply object from mesh fail!!')
    else:
        node.get_logger().info('success!')

def attach_hand(node):
    attach_hand_cli = node.create_client(AttachHand, 'attach_hand')
    req = AttachHand.Request()
    req.resource_path = 'package://hello_moveit/cad/robotiq_gripper/robotiq_2F_adaptive_gripper_rough.STL'
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
    if future.result().is_success is not True:
        node.get_logger().error('attach hand fail!!')
    else:
        node.get_logger().info('success!')


def check_collision(node):
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
    if future.result().collision_pairs:
        node.get_logger().warn('\n'.join(f'{pairs}' for pairs in future.result().collision_pairs))
    else:
        node.get_logger().info('no collision')

def detach_hand(node):
    detach_hand_cli = node.create_client(DetachHand, 'detach_hand')
    req = DetachHand.Request()
    req.object_id = 'robotiq_hand'

    while not detach_hand_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('detach hand service not ready, sleep 1sec')
    future = detach_hand_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result().is_success is not True:
        node.get_logger().error('detach hand fail!!')
    else:
        node.get_logger().info('success!')

def plan_execute_poses(node, pose):
    plan_execute_poses_cli = node.create_client(PlanExecutePoses, 'plan_execute_poses')
    req = PlanExecutePoses.Request()
    req.velocity_scale = 1.0  # you can chage this to slow down robot
    req.poses = [pose]

    while not plan_execute_poses_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')

    future = plan_execute_poses_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result().err_code.val is not MoveItErrorCodes.SUCCESS:
        node.get_logger().error(f'fail!!MoveItErrorCode: {future.result().err_code.val}')
    else:
        node.get_logger().info('success!')

def plan_execute_cartesian_path(node, poses):
    plan_execute_cartesian_cli = node.create_client(PlanExecuteCartesianPath, 'plan_execute_cartesian_path')
    req = PlanExecuteCartesianPath.Request()
    req.velocity_scale = 1.0  # you can chage this to slow down robot
    req.poses = poses

    while not plan_execute_cartesian_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')

    future = plan_execute_cartesian_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if not future.result().is_success:
        node.get_logger().error('faill cartesian_path')
    else:
        node.get_logger().info('success!')


def main() -> None:
    rclpy.init()
    node = rclpy.create_node('oreore')

    apply_collision_object(node)
    apply_collision_object_from_mesh(node)
    attach_hand(node)
    check_collision(node)
    # send target pose1
    msg = Pose()
    msg.orientation.w = -0.5
    msg.orientation.x = 0.5
    msg.orientation.y = 0.5
    msg.orientation.z = -0.5
    msg.position.x = -0.696
    msg.position.y = 0.052
    msg.position.z = 0.464
    plan_execute_poses(node, msg)
    # send target pose2
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
