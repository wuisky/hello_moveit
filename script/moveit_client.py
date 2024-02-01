import time

from geometry_msgs.msg import Pose
from hello_moveit.srv import ApplyCollisionObject, PlanExecutePoses
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
import rclpy
from shape_msgs.msg import SolidPrimitive


def main()->None:
    rclpy.init()
    node = rclpy.create_node('oreore')
    apply_collision_object_cli = node.create_client(ApplyCollisionObject, 'apply_collision_object')
    obj = CollisionObject()
    obj.id = 'wall'
    obj.operation = CollisionObject.ADD
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
    obj.primitive_poses.append(obj_pose)
    while not apply_collision_object_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')
    req = ApplyCollisionObject.Request()
    req.object = obj

    future = apply_collision_object_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result().is_success is not True:
        node.get_logger().error('apply object fail!!')
    else:
        node.get_logger().info('success!')



    plan_execute_poses_cli = node.create_client(PlanExecutePoses, 'plan_execute_poses')
    # set request
    req = PlanExecutePoses.Request()
    req.velocity_scale = 1.0
    msg = Pose()
    msg.orientation.w = -0.5
    msg.orientation.x = 0.5
    msg.orientation.y = 0.5
    msg.orientation.z = -0.5
    msg.position.x = -0.44
    msg.position.y = 0.026
    msg.position.z = 0.68
    req.poses = [msg]

    while not plan_execute_poses_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')

    future = plan_execute_poses_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result().err_code.val is not MoveItErrorCodes.SUCCESS:
        node.get_logger().error(f'fail!!MoveItErrorCode: {future.result().err_code.val}')
    else:
        node.get_logger().info('success!')

    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
