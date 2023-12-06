#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using moveit::planning_interface::MoveGroupInterface;

void findClosestSolution(
  const std::vector<double> & current_q,
  const std::vector<moveit::core::VariableBounds> & joint_bonds,
  std::vector<double> & solution)
{
  for (std::size_t i = 0; i < current_q.size(); ++i) {
    //RCLCPP_INFO(logger, "Joint %s: %f %f", joint_name[i].c_str(), solution[i], current_q[i]);
    // find closest angle
    if (std::fabs(current_q[i] - solution[i]) > M_PI) {
      if (current_q[i] > solution[i] && solution[i] + 2 * M_PI < joint_bonds[i].max_position_) {
        solution[i] += 2 * M_PI;
      } else if (current_q[i] < solution[i] &&
        solution[i] - 2 * M_PI > joint_bonds[i].min_position_)
      {
        solution[i] -= 2 * M_PI;
      }
    }
    //RCLCPP_INFO(logger, "Joint %s: %f", joint_name[i].c_str(), solution[i]);
  }
}

void planAndExecuteJointValue(
  const std::shared_ptr<rclcpp::Node> node,
  const std::vector<double> & q, const int retry, MoveGroupInterface & move_group_interface)
{
  auto const logger = node->get_logger();
  move_group_interface.setJointValueTarget(q);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  [&]()->void {
    for (int i = retry; i > 0; i--) {
      if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_interface.execute(plan);
        return;
      }
      RCLCPP_WARN_STREAM(logger, "try to replan");
    }
    RCLCPP_ERROR_STREAM(logger, "planning fail");
  }();
}

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
  // auto const logger = rclcpp::get_logger("hello_moveit");
  auto const logger = node->get_logger();
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

  //auto robot_model = move_group_interface.getRobotModel();


  // set velocity
  move_group_interface.setMaxVelocityScalingFactor(1.0);

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
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  const moveit::core::JointModelGroup * joint_model_group =
    move_group_interface.getCurrentState()->getJointModelGroup(arm_group);

  // RCLCPP_INFO(logger, "Available Planning Groups:");
  // std::copy(
  //   move_group_interface.getJointModelGroupNames().begin(),
  //   move_group_interface.getJointModelGroupNames().end(),
  //   std::ostream_iterator<std::string>(std::cout, ", "));

  // Next get the current set of joint values for the group.
  std::vector<double> seed_state;
  current_state->copyJointGroupPositions(joint_model_group, seed_state);
  for (double q : seed_state) {
    RCLCPP_INFO_STREAM(logger, "seed_state q: " << q);
  }
  auto joint_name = joint_model_group->getActiveJointModelNames();
  std::vector<moveit::core::VariableBounds> joint_bonds;
  for (std::string name : joint_name) {
    RCLCPP_INFO_STREAM(logger, "jnt: " << name);
    auto joint_model = joint_model_group->getJointModel(name);
    // joint_model->getType(); // prismatic, revolute
    auto bonds = joint_model->getVariableBounds(name);
    RCLCPP_INFO_STREAM(logger, "max=" << bonds.max_position_ << ", min=" << bonds.min_position_);
    joint_bonds.push_back(bonds);
  }
  RCLCPP_INFO_STREAM(logger, "planning time: " << move_group_interface.getPlanningTime());

  // ////////////setFromIK/////////////////////
  // robot_model_loader::RobotModelLoader robot_model_loader(node);
  // const moveit::core::RobotModelPtr & kinematic_model = robot_model_loader.getModel();
  // //moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  // current_state->setToDefaultValues();
  // double timeout = 0.1;
  // const Eigen::Isometry3d & end_effector_state = current_state->getGlobalLinkTransform(
  //   "wrist_3_link");
  // bool found_ik = current_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // // Now, we can print out the IK solution (if found):
  // if (found_ik) {
  //   current_state->copyJointGroupPositions(joint_model_group, seed_state);
  //   for (std::size_t i = 0; i < seed_state.size(); ++i) {
  //     RCLCPP_INFO(logger, "Joint %s: %f", joint_name[i].c_str(), seed_state[i]);
  //   }
  // } else {
  //   RCLCPP_INFO(logger, "Did not find IK solution");
  // }

  //move_group_interface.allowReplanning(true);
  //move_group_interface.setReplanAttempts(5);

  // [&move_group_interface, &seed_state]() {
  //   seed_state[0] = 0.0; // shoulder_pan_joint
  //   seed_state[1] = -2.689; //  shoulder_lift_joint
  //   seed_state[2] = 0.7441; // elbow_joint
  //   seed_state[3] = -2.768; //  wrist_1_joint
  //   seed_state[4] = 1.57; //  wrist_2_joint
  //   seed_state[5] = 1.57; // wrist_3_joint
  //   move_group_interface.setJointValueTarget(seed_state);
  //   moveit::planning_interface::MoveGroupInterface::Plan plan;
  //   if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
  //     move_group_interface.execute(plan);
  //   }
  // } ();

  // [&move_group_interface, &seed_state, &logger]() {
  //   seed_state[0] = 0.13249; // shoulder_pan_joint
  //   seed_state[1] = -1.8636;  //  shoulder_lift_joint
  //   seed_state[2] = -1.8725;  // elbow_joint
  //   seed_state[3] = -0.977; //  wrist_1_joint
  //   seed_state[4] = 1.57; //  wrist_2_joint
  //   seed_state[5] = 1.703; // wrist_3_joint
  //   move_group_interface.setJointValueTarget(seed_state);
  //   moveit::planning_interface::MoveGroupInterface::Plan plan;
  //   [&]()->void {
  //     for (int i = 3; i > 0; i--) {
  //       if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
  //         move_group_interface.execute(plan);
  //         return;
  //       }
  //       RCLCPP_WARN_STREAM(logger, "try to replan");
  //     }
  //     RCLCPP_ERROR_STREAM(logger, "planning fail");
  //   }();
  // } ();


  // Set a target Pose
  auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 0.0;
      msg.orientation.x = 1.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.0;
      msg.position.x = -0.625;
      msg.position.y = 0.133;
      msg.position.z = 0.614;
      return msg;
    }();

  auto ik_solver = joint_model_group->getSolverInstance();
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err_code;
  RCLCPP_INFO_STREAM(logger, "timeout " << joint_model_group->getDefaultIKTimeout());
  // joint_model_group->printGroupInfo(std::cout);
  RCLCPP_INFO_STREAM(logger, "timeout " << ik_solver->getDefaultTimeout());

  ik_solver->getPositionIK(target_pose, seed_state, solution, err_code);
  //ik_solver->searchPositionIK(target_pose, seed_state, 0.05, solution, err_code);
  RCLCPP_INFO_STREAM(logger, "errcode: " << err_code.val);

  if (err_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_FATAL_STREAM(logger, "IKERROR, errcode:" << err_code.val);
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  findClosestSolution(seed_state, joint_bonds, solution);
  planAndExecuteJointValue(node, solution, 30, move_group_interface);


  // if (!planAndExecutePose(
  //     target_pose, draw_title, prompt, draw_trajectory_tool_path,
  //     move_group_interface))
  // {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }

  auto const target_pose2 = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 0.0;
      msg.orientation.x = 1.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.0;
      msg.position.x = -0.56;
      msg.position.y = 0.06;
      msg.position.z = 0.25;

      return msg;
    }();

  current_state = move_group_interface.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, seed_state);

  ik_solver->getPositionIK(target_pose2, seed_state, solution, err_code);
  //ik_solver->searchPositionIK(target_pose, seed_state, 0.05, solution, err_code);
  RCLCPP_INFO_STREAM(logger, "errcode: " << err_code.val);

  findClosestSolution(seed_state, joint_bonds, solution);
  // for (std::size_t i = 0; i < solution.size(); ++i) {
  //   RCLCPP_INFO(logger, "solu %s: %f", joint_name[i].c_str(), solution[i]);
  // }
  planAndExecuteJointValue(node, solution, 30, move_group_interface);


  // if (!planAndExecutePose(
  //     target_pose2, draw_title, prompt, draw_trajectory_tool_path,
  //     move_group_interface))
  // {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
