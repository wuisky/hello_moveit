#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit/kinematic_constraints/utils.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometric_shapes/shape_operations.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <tf2_eigen/tf2_eigen.hpp>


using moveit::planning_interface::MoveGroupInterface;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

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

void planAndExecuteCartesianPath(
  const std::shared_ptr<rclcpp::Node> node,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const int retry, MoveGroupInterface & move_group_interface)
{
  auto const logger = node->get_logger();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  // path planning bug when avoid_collision is true
  // i don't know why
  const bool avoid_collision = false;

  [&]()->void {
    for (int i = retry; i > 0; i--) {
      if (move_group_interface.computeCartesianPath(
          waypoints, eef_step, jump_threshold,
          trajectory, avoid_collision) > 0.9)
      {
        char input;
        std::cout << "Were you admitted? [y/n]" << std::endl;
        std::cin >> input;
        if (input == 'y') {
          {
            move_group_interface.execute(trajectory);
            return;
          }
        }
      }
      RCLCPP_WARN_STREAM(logger, "try to replan");
    }
    RCLCPP_ERROR_STREAM(logger, "planning fail");
  }();
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
        // char input;
        // std::cout << "Were you admitted? [y/n]" << std::endl;
        // std::cin >> input;
        // if (input == 'y')
        {

          {
            move_group_interface.execute(plan);
            return;
          }
        }
      }
      RCLCPP_WARN_STREAM(logger, "try to replan");
    }
    RCLCPP_ERROR_STREAM(logger, "planning fail");
  }();
}

bool planAndExecutePose(
  const std::shared_ptr<rclcpp::Node> node,
  const geometry_msgs::msg::Pose & target_pose,
  const moveit::core::JointModelGroup * joint_model_group,
  MoveGroupInterface & move_group_interface,
  moveit_visual_tools::MoveItVisualTools & visual_tools)
{
  rclcpp::QoS qos(rclcpp::KeepLast{1000});
  auto pub = node->create_publisher<moveit_msgs::msg::RobotTrajectory>("/ref_robotraj",
  qos);
  move_group_interface.setPoseTarget(target_pose);
  auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

  // Execute the plan
  if (success) {
    const moveit::core::LinkModel * ee_parent_link =
      joint_model_group->getLinkModel(move_group_interface.getEndEffectorLink());
    visual_tools.publishTrajectoryLine(plan.trajectory_, ee_parent_link, joint_model_group);
    visual_tools.trigger();
    move_group_interface.execute(plan);
    pub->publish(std::move(plan.trajectory_));
    return true;
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "plan failed");
    return false;
  }
}

void planAndExecuteSequencePath(
  const std::shared_ptr<rclcpp::Node> node,
  const moveit_msgs::msg::MotionSequenceRequest & seq_req,
  const moveit::core::JointModelGroup * joint_model_group,
  MoveGroupInterface & move_group_interface,
  moveit_visual_tools::MoveItVisualTools & visual_tools
)
{
  auto planning_client = node->create_client<moveit_msgs::srv::GetMotionSequence>(
    "/plan_sequence_path");
  auto request = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
  request->request = seq_req;
  auto future = planning_client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
    RCLCPP_INFO_STREAM(node->get_logger(), "future not comeback ");
  } else {
    auto result = future.get();
    RCLCPP_INFO_STREAM(
      node->get_logger(), "plan ok size:" << result->response.planned_trajectories.size());

    // Get the parent link of the end effector
    const moveit::core::LinkModel * ee_parent_link =
      joint_model_group->getLinkModel(move_group_interface.getEndEffectorLink());
    // basically planned_trajectories size is 1

    // // send FollowJointTrajectoryAction
    // auto robo_traj = result->response.planned_trajectories[0];
    // // for (auto & point: robo_traj.joint_trajectory.points) {
    // //   point.accelerations.clear();
    // //   point.velocities.clear();
    // // }

    // // Create the action client
    // auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
    //         node, "/joint_trajectory_controller/follow_joint_trajectory");
    // // Wait for the action server to be available
    // if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
    //   RCLCPP_ERROR(node->get_logger(), "Action server not available!");
    // }
    // auto goal = FollowJointTrajectory::Goal();
    // goal.trajectory = robo_traj.joint_trajectory;
    // auto future_goal_handle = action_client->async_send_goal(goal);
    // if (future_goal_handle.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
    //   RCLCPP_ERROR(node->get_logger(), "Action goal send error");
    // }
    // future_goal_handle.get();
    ///////////////////////////////////
    rclcpp::QoS qos(rclcpp::KeepLast{1000});
    auto pub = node->create_publisher<moveit_msgs::msg::RobotTrajectory>("/ref_robotraj",
               qos);

    // int i=0;
    for (auto & traj:result->response.planned_trajectories) {
      // plot vel
      for (auto & point: traj.joint_trajectory.points) {
        std::cout << point.time_from_start.sec + point.time_from_start.nanosec * 1E-9 << " ";
        for (auto const & vel :point.velocities) {
          std::cout << vel << " ";
        }
        // std::cout << std::endl;
        // // adjust timestamp
        // point.time_from_start.sec = i/10;
        // point.time_from_start.nanosec = i%10 * 1E8;
        // i++;
      }
      visual_tools.publishTrajectoryLine(traj, ee_parent_link, joint_model_group);
      visual_tools.trigger();
      // prompt("Press 'next' in the RvizVisualToolsGui window to plan");
      move_group_interface.execute(traj);
      pub->publish(std::move(traj));
    }
    visual_tools.deleteAllMarkers();
    ///////////////
    // for (auto & traj:result->response.planned_trajectories) {
    //   // clear accelerations
    //   for (auto & point: traj.joint_trajectory.points) {
    //     point.accelerations.clear();
    //     point.velocities.clear();
    //   }

    //   visual_tools.publishTrajectoryLine(traj, ee_parent_link, joint_model_group);
    //   visual_tools.trigger();
    //   // prompt("Press 'next' in the RvizVisualToolsGui window to plan");
    //   move_group_interface.execute(traj);
    // }


  }
}

void addCollisionObject(
  const MoveGroupInterface & move_group_interface,
  moveit_msgs::msg::CollisionObject & collision_object)
{
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.operation = collision_object.ADD;

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  auto ret = planning_scene_interface.applyCollisionObject(collision_object);
  std::cout << "ret add obj:" << ret << std::endl;
}

moveit_msgs::msg::MotionSequenceItem createMotionSequenceItem(
  const std::string group_name,
  const std::string link_name,
  const geometry_msgs::msg::Pose target_pose,
  const std::string planner_id,
  const double blend_radius)
{
  moveit_msgs::msg::MotionSequenceItem item;
  moveit_msgs::msg::Constraints constraints;
  moveit_msgs::msg::PositionConstraint position_constraint;
  position_constraint.header.frame_id = "world";
  position_constraint.link_name = link_name;
  item.req.group_name = group_name;
  item.req.planner_id = planner_id;
  item.req.max_velocity_scaling_factor = 0.1;
  item.req.max_acceleration_scaling_factor = 0.1;
  item.req.allowed_planning_time = 5.0;
  item.blend_radius = blend_radius;
  auto stamped_target_pose = [&target_pose] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose = target_pose;
      return msg;
    }();
  item.req.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(link_name, stamped_target_pose));

  return item;
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
  // const std::string base_link = "base_link";
  const std::string base_link = "world";

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = MoveGroupInterface(node, arm_group);
  const std::string end_effect_name = move_group_interface.getEndEffectorLink();
  move_group_interface.setPlanningPipelineId("ompl");

  // Construct and initialize MoveItVisualTools
  auto visual_tools =
    moveit_visual_tools::MoveItVisualTools{node, base_link,
    rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().x() = -1.0;
  namespace rvt = rviz_visual_tools;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


  //auto robot_model = move_group_interface.getRobotModel();

  // set velocity
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);
  move_group_interface.setPlanningTime(30.0);

  // Create a closure for updating the text in rviz
  // auto const draw_title = [&visual_tools](std::string text) {
  //     auto const text_pose = [] {
  //         auto msg = Eigen::Isometry3d::Identity();
  //         msg.translation().z() = 1.0;
  //         return msg;
  //       }();
  //     visual_tools.publishText(
  //       text_pose, text, rviz_visual_tools::WHITE,
  //       rviz_visual_tools::XLARGE);
  //     visual_tools.trigger();
  //   };
  auto const prompt = [&visual_tools](std::string text) {
      visual_tools.prompt(text);
      visual_tools.trigger();
    };
  // auto const draw_trajectory_tool_path =
  //   [&visual_tools,
  //     jmg = move_group_interface.getRobotModel()->getJointModelGroup(arm_group)](
  //   // jmg = move_group_interface.getCurrentState()->getJointModelGroup(arm_group)](
  //   auto const trajectory) {
  //     visual_tools.publishTrajectoryLine(trajectory, jmg);
  //     visual_tools.trigger();
  //   };

  RCLCPP_INFO_STREAM(logger, "frame_id: " << move_group_interface.getPlanningFrame());
  // Create collision object for the robot to avoid
  ////////back wall/////////////////
  // [&move_group_interface] {
  //   moveit_msgs::msg::CollisionObject collision_object;
  //   collision_object.id = "wall1";
  //   shape_msgs::msg::SolidPrimitive primitive;

  //   // Define the size of the box in meters
  //   primitive.type = primitive.BOX;
  //   primitive.dimensions.resize(3);
  //   primitive.dimensions[primitive.BOX_X] = 0.1;
  //   primitive.dimensions[primitive.BOX_Y] = 1.0;
  //   primitive.dimensions[primitive.BOX_Z] = 1.0;
  //   collision_object.primitives.push_back(primitive);

  //   // Define the pose of the box (relative to the frame_id)
  //   auto const obj_pose = [] {
  //       geometry_msgs::msg::Pose obj_pose;
  //       obj_pose.orientation.w = 1.0;
  //       obj_pose.orientation.x = 0.0;
  //       obj_pose.orientation.y = 0.0;
  //       obj_pose.orientation.z = 0.0;
  //       obj_pose.position.x = 0.3;
  //       obj_pose.position.y = 0.0;
  //       obj_pose.position.z = 0.5;
  //       return obj_pose;
  //     }();
  //   collision_object.primitive_poses.push_back(obj_pose);
  //   addCollisionObject(move_group_interface, collision_object);
  // }();
  ////////shelf///////
  // Create collision object for the robot to avoid
  // [&] {
  //   moveit_msgs::msg::CollisionObject collision_object;
  //   collision_object.id = "shelf";
  //   // add mesh from stl
  //   Eigen::Vector3d scale(0.001, 0.001, 0.001);
  //   std::string resource = "package://hello_moveit/cad/shelf_rev5/type1_slid_shelf_change_1.STL";
  //   // std::string resource = "package://hello_moveit/cad/new_shelf.stl";
  //   // std::string resource = "package://hello_moveit/cad/2f-140.stl";
  //   shapes::Mesh * m = shapes::createMeshFromResource(resource, scale);
  //   shape_msgs::msg::Mesh co_mesh;
  //   shapes::ShapeMsg co_mesh_msg;
  //   shapes::constructMsgFromShape(m, co_mesh_msg);
  //   co_mesh = boost::get<shape_msgs::msg::Mesh>(co_mesh_msg);
  //   collision_object.meshes.push_back(co_mesh);
  //   RCLCPP_INFO_STREAM(
  //     logger,
  //     "tra len:" << co_mesh.triangles.size() << ", verti len" << co_mesh.vertices.size() );

  //   auto const obj_pose = [] {
  //       geometry_msgs::msg::Pose obj_pose;
  //       obj_pose.orientation.w = 1.0;
  //       obj_pose.orientation.x = 0.0;
  //       obj_pose.orientation.y = 0.0;
  //       obj_pose.orientation.z = 0.0;
  //       obj_pose.position.x = -0.70932;
  //       obj_pose.position.y = 0.43848;
  //       obj_pose.position.z = -0.83605;

  //       return obj_pose;
  //     }();
  //   collision_object.mesh_poses.push_back(obj_pose);
  //   addCollisionObject(move_group_interface, collision_object);
  // }();
  ///////////attach hand///////////////////
  [&] {
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.0;
    // grab_pose.position.z = 0.05; // for cylinder

    // shape_msgs::msg::SolidPrimitive cylinder_primitive;
    // shape_msgs::msg::SolidPrimitive primitive;
    // cylinder_primitive.type = primitive.CYLINDER;
    // cylinder_primitive.dimensions.resize(2);
    // cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.10;
    // cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = "robotiq_hand";
    object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
    // object_to_attach.primitives.push_back(cylinder_primitive); //for cylinder
    // object_to_attach.primitive_poses.push_back(grab_pose); //for cylinder

    Eigen::Vector3d scale(0.001, 0.001, 0.001);
    std::string resource =
      "package://hello_moveit/cad/robotiq_gripper/robotiq_2F_adaptive_gripper_rough.STL";
    shapes::Mesh * m = shapes::createMeshFromResource(resource, scale);
    shape_msgs::msg::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);
    co_mesh = boost::get<shape_msgs::msg::Mesh>(co_mesh_msg);
    object_to_attach.meshes.push_back(co_mesh);
    object_to_attach.mesh_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;

    // allow hand mesh collision with wrist_3_link
    moveit_msgs::msg::AttachedCollisionObject acobj;
    acobj.link_name = move_group_interface.getEndEffectorLink();
    acobj.object = object_to_attach;

    RCLCPP_INFO_STREAM(logger, "acobj linkname:" << acobj.link_name);//tool0
    std::vector<std::string> touch_links;
    touch_links.push_back("wrist_3_link");
    acobj.touch_links = touch_links;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyAttachedCollisionObject(acobj);
  }();

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
  RCLCPP_INFO_STREAM(logger, "planning time: " << move_group_interface.getPlanningTime() <<
    ", end_effect=" << move_group_interface.getEndEffectorLink());

  // ////////////setFromIK/////////////////////
  // robot_model_loader::RobotModelLoader robot_model_loader(node);
  // const moveit::core::RobotModelPtr & kinematic_model = robot_model_loader.getModel();
  // //moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  // current_state->setToDefaultValues();
  // double timeout = 0.1;
  // const Eigen::Isometry3d & end_effector_state = current_state->getGlobalLinkTransform(
  //   move_group_interface.getEndEffectorLink());
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

  auto ik_solver = joint_model_group->getSolverInstance();
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes err_code;
  RCLCPP_INFO_STREAM(logger, "timeout " << joint_model_group->getDefaultIKTimeout());
  // joint_model_group->printGroupInfo(std::cout);
  RCLCPP_INFO_STREAM(logger, "timeout " << ik_solver->getDefaultTimeout());

  // Set a target Pose
  auto target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 0.0;
      msg.orientation.x = 0.707;
      msg.orientation.y = 0.707;
      msg.orientation.z = 0.0;
      // msg.orientation.w = -0.5;
      // msg.orientation.x = 0.5;
      // msg.orientation.y = 0.5;
      // msg.orientation.z = -0.5;
      msg.position.x = -0.696;
      msg.position.y = 0.052;
      msg.position.z = 0.464;
      return msg;
    }();

  // auto const offset = [] {
  //     geometry_msgs::msg::Pose msg;
  //     msg.orientation.w = 1.0;
  //     msg.orientation.x = 0.0;
  //     msg.orientation.y = 0.0;
  //     msg.orientation.z = 0.0;
  //     msg.position.x = 0.0;
  //     msg.position.y = 0.0;
  //     msg.position.z = 0.07;
  //     return msg;
  //   }();

  // Eigen::Affine3d target_pose_aff, offset_aff;
  // tf2::fromMsg(target_pose, target_pose_aff  );
  // tf2::fromMsg(offset, offset_aff);
  // auto new_pose = tf2::toMsg(target_pose_aff * offset_aff.inverse());
  // ik_solver->getPositionIK(new_pose, seed_state, solution, err_code);

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

  for (std::size_t i = 0; i < solution.size(); ++i) {
    RCLCPP_INFO(logger, "Joint %s: %f", joint_name[i].c_str(), solution[i]);
  }


  planAndExecuteJointValue(node, solution, 30, move_group_interface);

  current_state = move_group_interface.getCurrentState(10);
  const Eigen::Isometry3d & end_effector_state = current_state->getGlobalLinkTransform(
    move_group_interface.getEndEffectorLink());
  RCLCPP_INFO_STREAM(logger, "tcp:\n " << end_effector_state.matrix());

  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  //////////////////////////

  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  // move_group_interface.setPlannerId("LIN");
  move_group_interface.setPlannerId("PTP");

  // // auto const target_pose2 = [] {
  // auto target_pose2 = [] {
  //     geometry_msgs::msg::Pose msg;
  //     // msg.orientation.w = -0.5;
  //     // msg.orientation.x = 0.5;
  //     // msg.orientation.y = 0.5;
  //     // msg.orientation.z = -0.5;
  //     msg.orientation.w = 0.0;
  //     msg.orientation.x = 0.707;
  //     msg.orientation.y = 0.707;
  //     msg.orientation.z = 0.0;

  //     msg.position.x = -0.696;
  //     // msg.position.y = 0.052;
  //     // msg.position.z = 0.154;
  //     msg.position.y = 0.352;
  //     msg.position.z = 0.464;

  //     return msg;
  //   }();

  // current_state = move_group_interface.getCurrentState(10);
  // current_state->copyJointGroupPositions(joint_model_group, seed_state);
  // ik_solver->getPositionIK(target_pose2, seed_state, solution, err_code);
  // //ik_solver->searchPositionIK(target_pose, seed_state, 0.05, solution, err_code);

  // findClosestSolution(seed_state, joint_bonds, solution);
  // planAndExecuteJointValue(node, solution, 30, move_group_interface);
  // fk example
  current_state->setJointGroupActivePositions(joint_model_group, solution);
  auto end_effector_state2 = current_state->getGlobalLinkTransform(
    move_group_interface.getEndEffectorLink());
  RCLCPP_INFO_STREAM(logger, "tcp:\n " << end_effector_state2.matrix());

  auto cpose = move_group_interface.getCurrentPose();
  RCLCPP_INFO_STREAM(logger, "tcp:\n " << cpose.pose.position.x <<
    " " << cpose.pose.position.y << " " << cpose.pose.position.z);
  target_pose = move_group_interface.getCurrentPose().pose;

  // //pilz LIN
  // target_pose.position.y += 0.3;
  // planAndExecutePose(node, target_pose, joint_model_group,
  //                     move_group_interface, visual_tools);
  // pilz
  {
    moveit_msgs::msg::MotionSequenceRequest seq_req;
    moveit_msgs::msg::MotionSequenceItem item;
    target_pose.position.y += 0.3;
    item = createMotionSequenceItem(arm_group, end_effect_name, target_pose,
                               //"LIN",
                               "PTP",
                                0.2);
    seq_req.items.push_back(item);

    target_pose.position.z -= 0.3;
    item = createMotionSequenceItem(arm_group, end_effect_name, target_pose,
                               //"LIN",
                               "PTP",
                               0.0);
    seq_req.items.push_back(item);

    // target_pose.position.y -= 0.3;
    // item = createMotionSequenceItem(arm_group, end_effect_name, target_pose,
    //                            "LIN",
    //                             0.0);
    // seq_req.items.push_back(item);

    planAndExecuteSequencePath(node, seq_req, joint_model_group,
                                                move_group_interface,
      visual_tools);
  }
  ////////////////

  // for (int i = 0; i < 30; i++) {
  //   {
  //     moveit_msgs::msg::MotionSequenceRequest seq_req;
  //     moveit_msgs::msg::MotionSequenceItem item;
  //     target_pose.position.z += 0.3;
  //     item = createMotionSequenceItem(arm_group, "tool0", target_pose,
  //                                  "pilz_industrial_motion_planner",
  //                              "LIN", 0.1);
  //     seq_req.items.push_back(item);

  //     target_pose.position.y -= 0.6;
  //     item = createMotionSequenceItem(arm_group, "tool0", target_pose,
  //                                  "pilz_industrial_motion_planner",
  //                              "LIN", 0.1);
  //     seq_req.items.push_back(item);

  //     target_pose.position.z -= 0.3;
  //     item = createMotionSequenceItem(arm_group, "tool0", target_pose,
  //                                  "pilz_industrial_motion_planner",
  //                              "LIN", 0.0);
  //     seq_req.items.push_back(item);

  //     planAndExecuteSequencePath(node, seq_req, joint_model_group, move_group_interface,
  //     visual_tools);
  //   }
  //   // prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  //   {
  //     moveit_msgs::msg::MotionSequenceRequest seq_req;
  //     moveit_msgs::msg::MotionSequenceItem item;
  //     target_pose.position.z += 0.3;
  //     item = createMotionSequenceItem(arm_group, "tool0", target_pose,
  //                                  "pilz_industrial_motion_planner",
  //                              "LIN", 0.1);
  //     seq_req.items.push_back(item);

  //     target_pose.position.y += 0.6;
  //     item = createMotionSequenceItem(arm_group, "tool0", target_pose,
  //                                  "pilz_industrial_motion_planner",
  //                              "LIN", 0.1);
  //     seq_req.items.push_back(item);

  //     target_pose.position.z -= 0.3;
  //     item = createMotionSequenceItem(arm_group, "tool0", target_pose,
  //                                  "pilz_industrial_motion_planner",
  //                              "LIN", 0.0);
  //     seq_req.items.push_back(item);

  //     planAndExecuteSequencePath(node, seq_req, joint_model_group, move_group_interface,
  //     visual_tools);
  //   }
  // }

  // // remove shelf.
  // std::vector<std::string> object_ids;
  // object_ids.push_back("shelf");
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // planning_scene_interface.removeCollisionObjects(object_ids);

  // //cartesian interp example
  // std::vector<geometry_msgs::msg::Pose> waypoints;
  // auto target_pose3(target_pose2);
  // target_pose3.position.y += 0.2;
  // waypoints.push_back(target_pose3);
  // target_pose3.position.y -= 0.2;
  // waypoints.push_back(target_pose3);
  // planAndExecuteCartesianPath(node, waypoints, 30, move_group_interface);

  //  detachhand
  move_group_interface.detachObject("robotiq_hand");

  std::vector<std::string> object_ids;
  object_ids.push_back("robotiq_hand");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.removeCollisionObjects(object_ids);
  // current_state->clearAttachedBody("robotiq_hand");

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
