#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometric_shapes/shape_operations.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <tf2_eigen/tf2_eigen.hpp>


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
        char input;
        std::cout << "Were you admitted? [y/n]" << std::endl;
        std::cin >> input;
        if (input == 'y') {
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

void addCollisionObject(
  const MoveGroupInterface & move_group_interface,
  moveit_msgs::msg::CollisionObject & collision_object,
  planning_scene::PlanningScene & planning_scene)
{
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.operation = collision_object.ADD;
  // std::cout << "id " << collision_object.header.frame_id << std::endl; //world
  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  auto ret = planning_scene_interface.applyCollisionObject(collision_object);

  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object); //moveit_msgs::msg::CollisionObject collision_object;
  auto ps = moveit_msgs::msg::PlanningScene();
  ps.world = psw;
  ps.is_diff = true;
  planning_scene.setPlanningSceneMsg(ps);
  // planning_scene.setPlanningSceneDiffMsg()

  std::cout << "ret add obj:" << ret << std::endl;
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

  RCLCPP_INFO_STREAM(logger, "frame_id: " << move_group_interface.getPlanningFrame());
  // Create collision object for the robot to avoid
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr & kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  ////////back wall/////////////////
  [&move_group_interface, &planning_scene] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "wall1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.0;
    primitive.dimensions[primitive.BOX_Z] = 1.0;
    collision_object.primitives.push_back(primitive);

    // Define the pose of the box (relative to the frame_id)
    auto const obj_pose = [] {
        geometry_msgs::msg::Pose obj_pose;
        obj_pose.orientation.w = -0.707;
        obj_pose.orientation.x = 0.0;
        obj_pose.orientation.y = 0.0;
        obj_pose.orientation.z = 0.707;
        obj_pose.position.x = 0.3;
        //obj_pose.position.y = 0.45;
        obj_pose.position.y = 0.05;
        obj_pose.position.z = 1.1;
        return obj_pose;
      }();
    collision_object.primitive_poses.push_back(obj_pose);
    addCollisionObject(move_group_interface, collision_object, planning_scene);
  }();
  ////////shelf///////
  // Create collision object for the robot to avoid
  [&] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "shelf";
    // add mesh from stl
    Eigen::Vector3d scale(0.001, 0.001, 0.001);
    std::string resource = "package://hello_moveit/cad/shelf_rev5/type1_slid_shelf_change_1.STL";
    // std::string resource = "package://hello_moveit/cad/new_shelf.stl";
    // std::string resource = "package://hello_moveit/cad/2f-140.stl";
    shapes::Mesh * m = shapes::createMeshFromResource(resource, scale);
    shape_msgs::msg::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);
    co_mesh = boost::get<shape_msgs::msg::Mesh>(co_mesh_msg);
    collision_object.meshes.push_back(co_mesh);
    RCLCPP_INFO_STREAM(
      logger,
      "tra len:" << co_mesh.triangles.size() << ", verti len" << co_mesh.vertices.size() );

    auto const obj_pose = [] {
        geometry_msgs::msg::Pose obj_pose;
        obj_pose.orientation.w = 1.0;
        obj_pose.orientation.x = 0.0;
        obj_pose.orientation.y = 0.0;
        obj_pose.orientation.z = 0.0;
        obj_pose.position.x = -0.70932;
        obj_pose.position.y = 0.43848;
        obj_pose.position.z = -0.83605;

        return obj_pose;
      }();
    collision_object.mesh_poses.push_back(obj_pose);
    addCollisionObject(move_group_interface, collision_object, planning_scene);
  }();
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
    // addCollisionObject(move_group_interface, object_to_attach);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //planning_scene_interface.applyCollisionObject(object_to_attach);


    // allow hand mesh collision with wrist_3_link
    moveit_msgs::msg::AttachedCollisionObject acobj;
    acobj.link_name = move_group_interface.getEndEffectorLink();
    acobj.object = object_to_attach;
    std::vector<std::string> touch_links;
    touch_links.push_back("wrist_3_link");
    acobj.touch_links = touch_links;

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
  RCLCPP_INFO_STREAM(logger, "planning time: " << move_group_interface.getPlanningTime());

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
  auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      // msg.orientation.w = 0.0;
      // msg.orientation.x = -0.707;
      // msg.orientation.y = 0.0;
      // msg.orientation.z = 0.707;
      msg.orientation.w = -0.5;
      msg.orientation.x = 0.5;
      msg.orientation.y = 0.5;
      msg.orientation.z = -0.5;
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

  // ik_solver->getPositionIK(target_pose, seed_state, solution, err_code);
  // //ik_solver->searchPositionIK(target_pose, seed_state, 0.05, solution, err_code);
  // RCLCPP_INFO_STREAM(logger, "errcode: " << err_code.val);

  // if (err_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
  //   RCLCPP_FATAL_STREAM(logger, "IKERROR, errcode:" << err_code.val);
  //   rclcpp::shutdown();
  //   spinner.join();
  //   return 1;
  // }
  // findClosestSolution(seed_state, joint_bonds, solution);
  // planAndExecuteJointValue(node, solution, 30, move_group_interface);

  // current_state = move_group_interface.getCurrentState(10);
  // const Eigen::Isometry3d & end_effector_state = current_state->getGlobalLinkTransform(
  //   move_group_interface.getEndEffectorLink());
  // RCLCPP_INFO_STREAM(logger, "tcp:\n " << end_effector_state.matrix());

  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = "ur_manipulator";
  collision_request.contacts = true;
  // collision_request.cost = true;
  // collision_request.max_cost_sources = collision_request.max_contacts;
  // collision_request.max_contacts *= collision_request.max_contacts;

  collision_request.max_contacts = 1;
  collision_detection::CollisionResult collision_result;
  current_state = move_group_interface.getCurrentState(10);
  moveit::core::RobotState copied_state = planning_scene.getCurrentState();
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  planning_scene.checkCollision(collision_request, collision_result, copied_state);

  RCLCPP_INFO_STREAM(
    logger, "Test 7: Current state is " << (collision_result.collision ? "in" : "not in")
                                        << " collision");
  auto collision = planning_scene.isStateColliding();
  RCLCPP_INFO_STREAM(
    logger, "Test 8: Current state is " << (collision ? "in" : "not in")
                                        << " collision");
  // move_group_interface.detachObject("robotiq_hand");

  // //////////////////////////
  // auto const target_pose2 = [] {
  //     geometry_msgs::msg::Pose msg;
  //     msg.orientation.w = -0.5;
  //     msg.orientation.x = 0.5;
  //     msg.orientation.y = 0.5;
  //     msg.orientation.z = -0.5;
  //     msg.position.x = -0.696;
  //     msg.position.y = 0.0519;
  //     msg.position.z = 0.154;

  //     return msg;
  //   }();

  // current_state = move_group_interface.getCurrentState(10);
  // current_state->copyJointGroupPositions(joint_model_group, seed_state);
  // ik_solver->getPositionIK(target_pose2, seed_state, solution, err_code);
  // //ik_solver->searchPositionIK(target_pose, seed_state, 0.05, solution, err_code);

  // findClosestSolution(seed_state, joint_bonds, solution);
  // planAndExecuteJointValue(node, solution, 30, move_group_interface);
  // // fk example
  // current_state->setJointGroupActivePositions(joint_model_group, solution);
  // auto end_effector_state2 = current_state->getGlobalLinkTransform(
  //   move_group_interface.getEndEffectorLink());
  // RCLCPP_INFO_STREAM(logger, "tcp:\n " << end_effector_state2.matrix());

  // // // remove shelf.
  // // std::vector<std::string> object_ids;
  // // object_ids.push_back("shelf");
  // // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // // planning_scene_interface.removeCollisionObjects(object_ids);

  // //cartesian interp example
  // std::vector<geometry_msgs::msg::Pose> waypoints;
  // auto target_pose3(target_pose2);
  // target_pose3.position.y += 0.2;
  // waypoints.push_back(target_pose3);
  // target_pose3.position.y -= 0.2;
  // waypoints.push_back(target_pose3);
  // planAndExecuteCartesianPath(node, waypoints, 30, move_group_interface);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
