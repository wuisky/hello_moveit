#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/collision_object.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hello_moveit/srv/apply_collision_object.hpp"
#include "hello_moveit/srv/apply_collision_object_from_mesh.hpp"
#include "hello_moveit/srv/attach_hand.hpp"
#include "hello_moveit/srv/check_collision.hpp"
#include "hello_moveit/srv/plan_execute_poses.hpp"


using moveit::planning_interface::MoveGroupInterface;

class MoveitClient
{
public:
  MoveitClient(
    const std::string arm_group,
    std::shared_ptr<rclcpp::Node> node,
    MoveGroupInterface & move_group)
  : arm_group_(arm_group), node_(node), move_group_(move_group),
    logger_(node->get_logger()) {}

  void initServer()
  {
    joint_model_group_ =
      move_group_.getCurrentState()->getJointModelGroup(arm_group_);

    //assign joint bonds
    auto joint_name = joint_model_group_->getActiveJointModelNames();
    for (std::string name : joint_name) {
      RCLCPP_INFO_STREAM(logger_, "jnt: " << name);
      auto joint_model = joint_model_group_->getJointModel(name);
      // joint_model->getType(); // prismatic, revolute
      auto bonds = joint_model->getVariableBounds(name);
      RCLCPP_INFO_STREAM(logger_, "max=" << bonds.max_position_ << ", min=" << bonds.min_position_);
      joint_bonds_.push_back(bonds);
    }

    ik_solver_ = joint_model_group_->getSolverInstance();

    // start service
    plan_execute_poses_srv_ = node_->create_service<hello_moveit::srv::PlanExecutePoses>(
      "plan_execute_poses",
      std::bind(
        &MoveitClient::planExecutePosesCB, this, std::placeholders::_1,
        std::placeholders::_2));

    apply_collision_object_srv_ = node_->create_service<hello_moveit::srv::ApplyCollisionObject>(
      "apply_collision_object",
      std::bind(
        &MoveitClient::applyCollisionObjectCB, this, std::placeholders::_1,
        std::placeholders::_2));

    apply_collision_object_from_mesh_srv_ =
      node_->create_service<hello_moveit::srv::ApplyCollisionObjectFromMesh>(
      "apply_collision_object_from_mesh",
      std::bind(
        &MoveitClient::applyCollisionObjectFromMeshCB, this, std::placeholders::_1,
        std::placeholders::_2));

    attach_hand_srv_ = node_->create_service<hello_moveit::srv::AttachHand>(
      "attach_hand",
      std::bind(
        &MoveitClient::attachHandCB, this, std::placeholders::_1,
        std::placeholders::_2));

    current_state_ = move_group_.getCurrentState(3);
    std::vector<double> seed_state, solution;
    current_state_->copyJointGroupPositions(joint_model_group_, seed_state);
    for (auto const & q:seed_state) {
      RCLCPP_INFO_STREAM(logger_, "q_init: " << q);
    }
    RCLCPP_INFO(logger_, "service is created");
  }

  void planExecutePosesCB(
    const std::shared_ptr<hello_moveit::srv::PlanExecutePoses::Request> request,
    std::shared_ptr<hello_moveit::srv::PlanExecutePoses::Response> respons)
  {
    RCLCPP_INFO(logger_, "service is called");
    const auto & target_pose = request->poses[0];
    RCLCPP_INFO_STREAM(logger_, "input " << target_pose.position.x);
    current_state_ = move_group_.getCurrentState(3); //this is necessary
    std::vector<double> seed_state, solution;
    current_state_->copyJointGroupPositions(joint_model_group_, seed_state);
    for (auto const & q:seed_state) {
      RCLCPP_INFO_STREAM(logger_, "q: " << q);
    }
    ik_solver_->getPositionIK(target_pose, seed_state, solution, respons->err_code);

    if (respons->err_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(logger_, "IKERROR, errcode:" << respons->err_code.val);
      return;
    }

    move_group_.setMaxVelocityScalingFactor(request->velocity_scale);
    findClosestSolution_(seed_state, solution);
    for (size_t i = 0; i < solution.size(); ++i) {
      RCLCPP_INFO_STREAM(logger_, "q[" << i << "]=" << solution[i]);
    }
    respons->err_code = planAndExecuteJointValue_(solution, 30);
    RCLCPP_INFO(logger_, "service is retrun");
  }

  void applyCollisionObjectCB(
    const std::shared_ptr<hello_moveit::srv::ApplyCollisionObject::Request> request,
    std::shared_ptr<hello_moveit::srv::ApplyCollisionObject::Response> respons)
  {
    if (request->object.header.frame_id.empty()) {
      request->object.header.frame_id = move_group_.getPlanningFrame();
    }
    respons->is_success = planning_scene_interface_.applyCollisionObject(request->object);
  }

  void applyCollisionObjectFromMeshCB(
    const std::shared_ptr<hello_moveit::srv::ApplyCollisionObjectFromMesh::Request> request,
    std::shared_ptr<hello_moveit::srv::ApplyCollisionObjectFromMesh::Response> respons)
  {
    auto object = loadObjectFromMesh_<hello_moveit::srv::ApplyCollisionObjectFromMesh::Request>(
      request);

    object.operation = request->operation;
    if (request->frame_id.empty()) {
      object.header.frame_id = move_group_.getPlanningFrame();
    } else {
      object.header.frame_id = request->frame_id;
    }

    respons->is_success = planning_scene_interface_.applyCollisionObject(object);
  }

  void attachHandCB(
    const std::shared_ptr<hello_moveit::srv::AttachHand::Request> request,
    std::shared_ptr<hello_moveit::srv::AttachHand::Response> respons)
  {
    auto object = loadObjectFromMesh_<hello_moveit::srv::AttachHand::Request>(request);

    object.header.frame_id = move_group_.getEndEffectorLink();
    object.operation = object.ADD;
    moveit_msgs::msg::AttachedCollisionObject acobj;
    acobj.link_name = move_group_.getEndEffectorLink();
    acobj.object = object;
    acobj.touch_links = request->touch_links;
    respons->is_success = planning_scene_interface_.applyAttachedCollisionObject(acobj);
  }

  void checkCollisionCB(
    const std::shared_ptr<hello_moveit::srv::CheckCollision::Request> request,
    std::shared_ptr<hello_moveit::srv::CheckCollision::Response> respons)
  {

  }

private:
  template<typename RequestType>
  static moveit_msgs::msg::CollisionObject loadObjectFromMesh_(
    const std::shared_ptr<RequestType> request)
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = request->object_id;
    Eigen::Vector3d scale(request->scale, request->scale, request->scale);
    shapes::Mesh * m = shapes::createMeshFromResource(request->resource_path, scale);
    shape_msgs::msg::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);
    co_mesh = boost::get<shape_msgs::msg::Mesh>(co_mesh_msg);
    object.meshes.push_back(co_mesh);
    object.mesh_poses.push_back(request->pose);
    return object;
  }


  // this function my not necessary depends on kinematic plugin
  void findClosestSolution_(const std::vector<double> & current_q, std::vector<double> & solution)
  {
    for (std::size_t i = 0; i < current_q.size(); ++i) {
      //RCLCPP_INFO(logger, "Joint %s: %f %f", joint_name[i].c_str(), solution[i], current_q[i]);
      // find closest angle
      if (std::fabs(current_q[i] - solution[i]) > M_PI) {
        if (current_q[i] > solution[i] && solution[i] + 2 * M_PI < joint_bonds_[i].max_position_) {
          solution[i] += 2 * M_PI;
        } else if (current_q[i] < solution[i] &&
          solution[i] - 2 * M_PI > joint_bonds_[i].min_position_)
        {
          solution[i] -= 2 * M_PI;
        }
      }
      //RCLCPP_INFO(logger, "Joint %s: %f", joint_name[i].c_str(), solution[i]);
    }
  }

  moveit::core::MoveItErrorCode planAndExecuteJointValue_(
    const std::vector<double> & q,
    const int retry)
  {
    move_group_.setJointValueTarget(q);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode err;

    [&]()->void {
      for (int i = retry; i > 0; i--) {
        err = move_group_.plan(plan);
        if (err.val == moveit::core::MoveItErrorCode::SUCCESS) {
          char input;
          std::cout << "Were you admitted? [y/n]" << std::endl;
          std::cin >> input;
          if (input == 'y') {
            {
              move_group_.execute(plan);
              return;
            }
          }
        }
        RCLCPP_WARN_STREAM(logger_, "try to replan");
      }
      RCLCPP_ERROR_STREAM(logger_, "planning fail");
    }();

    return err;
  }

  const std::string arm_group_;
  std::shared_ptr<rclcpp::Node> node_;
  MoveGroupInterface & move_group_;
  rclcpp::Service<hello_moveit::srv::PlanExecutePoses>::SharedPtr
    plan_execute_poses_srv_;

  rclcpp::Service<hello_moveit::srv::ApplyCollisionObject>::SharedPtr
    apply_collision_object_srv_;

  rclcpp::Service<hello_moveit::srv::ApplyCollisionObjectFromMesh>::SharedPtr
    apply_collision_object_from_mesh_srv_;

  rclcpp::Service<hello_moveit::srv::AttachHand>::SharedPtr
    attach_hand_srv_;

  rclcpp::Service<hello_moveit::srv::CheckCollision>::SharedPtr
    check_collision_srv_;

  kinematics::KinematicsBaseConstPtr ik_solver_;
  moveit::core::RobotStatePtr current_state_;
  const moveit::core::JointModelGroup * joint_model_group_;
  std::vector<moveit::core::VariableBounds> joint_bonds_;
  rclcpp::Logger logger_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};


int main(int argc, char * argv[])
{
  // // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "moveit_client", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  //todo get this param from ros param
  const std::string kARM_GROUP = "ur_manipulator";

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});
  //auto spinner = std::thread([&node]() {rclcpp::spin(node);});

  // Create the MoveIt MoveGroup Interface
  auto move_group = MoveGroupInterface(node, kARM_GROUP);
  MoveitClient mc(kARM_GROUP, node, move_group);
  mc.initServer();

  // wait until SIGTERM
  spinner.join();
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
