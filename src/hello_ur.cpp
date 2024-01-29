#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometric_shapes/shape_operations.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>


#include <tf2_eigen/tf2_eigen.hpp>


using moveit::planning_interface::MoveGroupInterface;

void addCollisionObject(
  const MoveGroupInterface & move_group_interface,
  moveit::planning_interface::PlanningSceneInterface & planning_scene_interface,
  moveit_msgs::msg::CollisionObject & collision_object,
  planning_scene::PlanningScene & planning_scene)
{
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.operation = collision_object.ADD;
  // std::cout << "id " << collision_object.header.frame_id << std::endl; //world
  // Add the collision object to the scene

  auto ret = planning_scene_interface.applyCollisionObject(collision_object);

  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object); //moveit_msgs::msg::CollisionObject collision_object;
  auto ps = moveit_msgs::msg::PlanningScene();
  ps.world = psw;
  ps.is_diff = true;
  //planning_scene.setPlanningSceneMsg(ps);
  planning_scene.setPlanningSceneDiffMsg(ps);

  std::cout << "ret add obj:" << ret << std::endl;
}


// Function to convert moveit_msgs::msg::AttachedCollisionObject to moveit::core::AttachedBody
//moveit::core::AttachedBody convertToAttachedBody(
Eigen::Isometry3d convertPoseToIsometry3d(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d object_pose;
  object_pose.translation() << pose.position.x, pose.position.y, pose.position.z;
  object_pose.linear() = Eigen::Quaterniond(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z
  ).toRotationMatrix();

  return object_pose;
}


int main(int argc, char * argv[])
{
  // // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
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
  RCLCPP_INFO_STREAM(logger, "frame_id: " << move_group_interface.getPlanningFrame());
  // Create collision object for the robot to avoid
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr & kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ////////collision object import/////////////////
  [&move_group_interface, &planning_scene, &planning_scene_interface] {
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
        obj_pose.position.y = 0.35;
        //obj_pose.position.y = 0.15;
        obj_pose.position.z = 1.1;
        return obj_pose;
      }();
    collision_object.primitive_poses.push_back(obj_pose);
    addCollisionObject(
      move_group_interface, planning_scene_interface, collision_object,
      planning_scene);
  }();

  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  ///////////attach hand///////////////////
  [&] {
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.0;

    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = "robotiq_hand";
    object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();

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

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::AttachedCollisionObject acobj;
    acobj.link_name = move_group_interface.getEndEffectorLink();
    acobj.object = object_to_attach;
    std::vector<std::string> touch_links;
    touch_links.push_back("wrist_3_link");
    acobj.touch_links = touch_links;

    planning_scene_interface.applyAttachedCollisionObject(acobj);

    current_state = move_group_interface.getCurrentState(10);
    shapes::ShapePtr m_ptr(m);
    std::vector<shapes::ShapeConstPtr> shapes;
    shapes.push_back(m_ptr);
    Eigen::Isometry3d zero_pose(Eigen::Isometry3d::Identity());
    auto shape_pose = convertPoseToIsometry3d(grab_pose);
    EigenSTL::vector_Isometry3d shape_poses;
    shape_poses.push_back(shape_pose);

    current_state->attachBody(
      object_to_attach.id, zero_pose, shapes, shape_poses,
      touch_links, acobj.link_name);

  }();

  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = "ur_manipulator";
  collision_request.contacts = true;
  // collision_request.cost = true;
  // collision_request.max_cost_sources = collision_request.max_contacts;
  // collision_request.max_contacts *= collision_request.max_contacts;

  collision_request.max_contacts = 100;
  collision_detection::CollisionResult collision_result;

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  acm.print(std::cout);
  planning_scene.checkCollision(collision_request, collision_result, *current_state);
  RCLCPP_INFO_STREAM(
    logger, "Current state is " << (collision_result.collision ? "in" : "not in")
                                << " collision");

  if (collision_result.collision) {
    for (const auto & entry : collision_result.contacts) {
      const auto & key = entry.first;
      RCLCPP_INFO_STREAM(logger, "collision between:" << key.first << ", " << key.second);
    }
  }

  auto collision = planning_scene.isStateColliding();
  RCLCPP_INFO_STREAM(
    logger, "Current state is " << (collision ? "in" : "not in")
                                << " collision");

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
