import time

from geometry_msgs.msg import Pose
from hello_moveit.srv import PlanExecutePoses
from moveit_msgs.msg import MoveItErrorCodes
import rclpy


def main()->None:
    rclpy.init()
    node = rclpy.create_node('oreore')
    cli = node.create_client(PlanExecutePoses, 'plan_execute_poses')
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

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not ready, sleep 1sec')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result().err_code.val is not MoveItErrorCodes.SUCCESS:
        node.get_logger().error(f'fail!!MoveItErrorCode: {future.result().err_code.val}')
    else:
        node.get_logger().info('success!')

    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
