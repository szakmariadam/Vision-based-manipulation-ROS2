#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("pickplace_node_mgi", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string ARM_GROUP = "ur_arm";
  static const std::string GRIPPER_GROUP = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, ARM_GROUP);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, GRIPPER_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Adding object to the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm.getPlanningFrame();

  collision_object.id = "cube";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.05;
  primitive.dimensions[primitive.BOX_Y] = 0.05;
  primitive.dimensions[primitive.BOX_Z] = 0.05;

  geometry_msgs::msg::Pose cube_pose;
  cube_pose.orientation.w = 1.0;
  cube_pose.position.x = 0;
  cube_pose.position.y = 0;
  cube_pose.position.z = 1.05;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cube_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);

  // Go above the object
  // ^^^^^^^^^^^^^^^^^^^
  geometry_msgs::msg::Pose above_object;
  above_object.orientation.w = 0;
  above_object.position.x = 0;
  above_object.position.y = 0;
  above_object.position.z = 1.25;
  above_object.orientation.x = 0.707;
  above_object.orientation.y = 0.707;
  move_group_arm.setPoseTarget(above_object);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group_arm.plan(my_plan);

  move_group_arm.setMaxVelocityScalingFactor(0.8);
  move_group_arm.setMaxAccelerationScalingFactor(0.8);
  
  move_group_arm.move();

  // Approach object
  // ^^^^^^^^^^^^^^^

  std::vector<geometry_msgs::msg::Pose> waypoints;

  auto cartesian_target_pose = above_object;

  waypoints.push_back(cartesian_target_pose);

  cartesian_target_pose.position.z -= 0.2;
  waypoints.push_back(cartesian_target_pose);

  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;

  move_group_arm.computeCartesianPath(waypoints, eef_step, trajectory);

  move_group_arm.execute(trajectory);

  // Attach object
  // ^^^^^^^^^^^^^

  std::vector<std::string> touch_links;
  touch_links.push_back("rh_p12_rn_l1");
  touch_links.push_back("rh_p12_rn_r1");
  touch_links.push_back("rh_p12_rn_l2");
  touch_links.push_back("rh_p12_rn_r2");
  move_group_arm.attachObject("cube", "end_effector_link", touch_links);

  // Close gripper
  // ^^^^^^^^^^^^^

  move_group_gripper.setMaxVelocityScalingFactor(1);
  move_group_gripper.setMaxAccelerationScalingFactor(1);

  move_group_gripper.setNamedTarget("Close");
  move_group_gripper.move();

  // Pick up object
  // ^^^^^^^^^^^^^^

  waypoints.clear();

  waypoints.push_back(cartesian_target_pose);

  cartesian_target_pose.position.z += 0.2;
  waypoints.push_back(cartesian_target_pose);


  move_group_arm.computeCartesianPath(waypoints, eef_step, trajectory);

  move_group_arm.execute(trajectory);

  // Go to place goal
  // ^^^^^^^^^^^^^^^^
  geometry_msgs::msg::Pose place_pose;
  place_pose.orientation.w = 0;
  place_pose.position.x = -0.2;
  place_pose.position.y = 0;
  place_pose.position.z = 1.05;
  place_pose.orientation.x = 0.707;
  place_pose.orientation.y = 0.707;
  move_group_arm.setPoseTarget(place_pose);

  move_group_arm.plan(my_plan);
  
  move_group_arm.move();

  // Open gripper
  // ^^^^^^^^^^^^
  move_group_gripper.setNamedTarget("Open");
  move_group_gripper.move();

  // Detach object
  // ^^^^^^^^^^^^^
  move_group_arm.detachObject("cube");

  // retreat
  // ^^^^^^^

  waypoints.clear();

  waypoints.push_back(place_pose);

  cartesian_target_pose = place_pose;

  cartesian_target_pose.position.z += 0.2;
  waypoints.push_back(cartesian_target_pose);

  move_group_arm.computeCartesianPath(waypoints, eef_step, trajectory);

  move_group_arm.execute(trajectory);

  // Go Home
  // ^^^^^^^
  move_group_arm.setNamedTarget("Home");
  move_group_arm.move();

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  return 0;
}