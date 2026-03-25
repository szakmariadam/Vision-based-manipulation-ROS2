#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_api_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("motion_planning_pipeline_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));

  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(node, robot_model_loader));
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("ur_arm");

  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));

  /* We can also use visual_tools to wait for user input */
  rclcpp::sleep_for(std::chrono::milliseconds(2000));

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  // Add object
  // ^^^^^^^^^^

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "world";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::msg::Pose object_pose;
  object_pose.position.z = 1.05;
  object_pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.05;
  primitive.dimensions[1] = 0.05;
  primitive.dimensions[2] = 0.05;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(object_pose);

  attached_object.object.operation = attached_object.object.ADD;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);

  // Go above object
  // ^^^^^^^^^^^^^^^
  
  //get object position
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  auto object_poses = planning_scene_interface.getObjectPoses({"box"});

  geometry_msgs::msg::Pose get_object_pose = object_poses["box"];

  planning_interface::MotionPlanRequest req;
  req.pipeline_id = "ompl";
  req.planner_id = "RRTConnectkConfigDefault";
  req.allowed_planning_time = 1.0;
  req.max_velocity_scaling_factor = 0.8;
  req.max_acceleration_scaling_factor = 0.8;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.z = get_object_pose.position.z + 0.2;
  pose.pose.position.x = get_object_pose.position.x;
  pose.pose.position.y = get_object_pose.position.y;
  pose.pose.orientation.x = 0.707;
  pose.pose.orientation.y = 0.707;
  pose.pose.orientation.w = 0;

  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  req.group_name = "ur_arm";
  moveit_msgs::msg::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("end_effector_link", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);

    if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code.val != res.error_code.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      rclcpp::shutdown();
      return -1;
    }
  }

  //execute
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  res.trajectory->getRobotTrajectoryMsg(trajectory_msg);

  trajectory_execution_manager::TrajectoryExecutionManagerPtr tem(
      new trajectory_execution_manager::TrajectoryExecutionManager(
          node, robot_model, psm->getStateMonitor()));

  tem->push(trajectory_msg);
  tem->execute();
  tem->waitForExecution();

  // approach object
  // ^^^^^^^^^^^^^^^

  pose.header.frame_id = "world";
  pose.pose.position.z = get_object_pose.position.z;
  pose.pose.position.x = get_object_pose.position.x;
  pose.pose.position.y = get_object_pose.position.y;
  pose.pose.orientation.x = 0.707;
  pose.pose.orientation.y = 0.707;
  pose.pose.orientation.w = 0;

  pose_goal =
      kinematic_constraints::constructGoalConstraints("end_effector_link", pose, tolerance_pose, tolerance_angle);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);

    if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code.val != res.error_code.SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
      rclcpp::shutdown();
      return -1;
    }
  }

  //execute
  res.trajectory->getRobotTrajectoryMsg(trajectory_msg);

  tem->push(trajectory_msg);
  tem->execute();
  tem->waitForExecution();
  
  rclcpp::shutdown();
  return 0;
}