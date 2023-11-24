#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using moveit::planning_interface::MoveGroupInterface;

bool planAndExecutePose(
  const geometry_msgs::msg::Pose & target_pose,
  const std::function<void(std::string)> draw_title,
  const std::function<void(std::string)> prompt,
  const std::function<void(moveit_msgs::msg::RobotTrajectory trajectory_msg)> draw_trajectory_tool_path,
  MoveGroupInterface & move_group_interface)
{
  move_group_interface.setPoseTarget(target_pose);
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  //moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    //moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    //moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
    return true;
  } else {
    draw_title("Planning Failed!");
    //moveit_visual_tools.trigger();
    return false;
  }
}

int main(int argc, char * argv[])
{
  // // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  const std::string arm_group = "ur_manipulator";
  const std::string base_link = "base_link";

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = MoveGroupInterface(node, arm_group);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
    moveit_visual_tools::MoveItVisualTools{node, base_link,
    rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](std::string text) {
      auto const text_pose = [] {
          auto msg = Eigen::Isometry3d::Identity();
          msg.translation().z() = 1.0;
          return msg;
        }();
      moveit_visual_tools.publishText(
        text_pose, text, rviz_visual_tools::WHITE,
        rviz_visual_tools::XLARGE);
      moveit_visual_tools.trigger();
    };
  auto const prompt = [&moveit_visual_tools](std::string text) {
      moveit_visual_tools.prompt(text);
      moveit_visual_tools.trigger();
    };
  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(arm_group)](
    auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      moveit_visual_tools.trigger();
    };

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = frame_id;
      collision_object.id = "box1";
      shape_msgs::msg::SolidPrimitive primitive;

      // Define the size of the box in meters
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.5;
      primitive.dimensions[primitive.BOX_Y] = 0.1;
      primitive.dimensions[primitive.BOX_Z] = 0.5;

      // Define the pose of the box (relative to the frame_id)
      geometry_msgs::msg::Pose box_pose;
      box_pose.orientation.w = 0.707;
      box_pose.orientation.x = 0.707;
      box_pose.orientation.y = 0.0;
      box_pose.orientation.z = 0.0;
      box_pose.position.x = -0.79;
      box_pose.position.y = 0.2;
      box_pose.position.z = 0.47;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      return collision_object;
    }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  ////////////////////////////////////////////
  // Set a target Pose
  auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 0.5;
      msg.orientation.x = -0.5;
      msg.orientation.y = -0.5;
      msg.orientation.z = 0.5;
      msg.position.x = -0.625;
      msg.position.y = 0.133;
      msg.position.z = 0.614;
      return msg;
    }();

  if (!planAndExecutePose(
      target_pose, draw_title, prompt, draw_trajectory_tool_path,
      move_group_interface))
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  auto const target_pose2 = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 0.5;
      msg.orientation.x = -0.5;
      msg.orientation.y = -0.5;
      msg.orientation.z = 0.5;
      msg.position.x = -0.7;
      msg.position.y = 0.178;
      msg.position.z = 0.186;

      return msg;
    }();

  if (!planAndExecutePose(
      target_pose2, draw_title, prompt, draw_trajectory_tool_path,
      move_group_interface))
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
