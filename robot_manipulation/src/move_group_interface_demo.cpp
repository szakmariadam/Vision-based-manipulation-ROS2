#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_interface_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "ur_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, 
    "world", 
    "rviz_visual_tools", 
    move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "test", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 0;
  target_pose1.position.x = 0;
  target_pose1.position.y = 0;
  target_pose1.position.z = 1.05;
  target_pose1.orientation.x = 0.707;
  target_pose1.orientation.y = 0.707;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.setMaxVelocityScalingFactor(1);
  move_group.setMaxAccelerationScalingFactor(1);
  
  //move_group.move();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  //move_group.setMaxVelocityScalingFactor(0.05);
  //move_group.setMaxAccelerationScalingFactor(0.05);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //move_group.move();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations

  auto current_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);

  auto cartesian_target_pose = current_pose;

  cartesian_target_pose.position.z -= 0.2;
  cartesian_target_pose.position.x -= 0.2;
  waypoints.push_back(cartesian_target_pose);

  cartesian_target_pose.position.z -= 0.2;
  waypoints.push_back(cartesian_target_pose);

  cartesian_target_pose.position.y -= 0.2;
  waypoints.push_back(cartesian_target_pose);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm,
  // which is why we will specify 0.01 as the max step in Cartesian translation.
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
  RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //move_group.execute(trajectory);
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

// Adding objects to the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // First, let's plan to another simple goal with no objects in the way.
  move_group.setPoseTarget(target_pose1);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Now, let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.3;
  primitive.dimensions[primitive.BOX_Y] = 1;
  primitive.dimensions[primitive.BOX_Z] = 0.6;

  // Define a pose for the box (specified relative to frame_id).
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = 1.7;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now, when we plan a trajectory it will avoid the obstacle.
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
  visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // Attaching objects to the robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // You can attach an object to the robot, so that it moves with the robot geometry.
  // This simulates picking up the object for the purpose of manipulating it.
  // The motion planning should avoid collisions between objects as well.
  moveit_msgs::msg::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";

  shape_msgs::msg::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.10;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.02;

  // We define the frame/pose for this cylinder so that it appears in the gripper.
  object_to_attach.header.frame_id = move_group.getEndEffectorLink();
  geometry_msgs::msg::Pose grab_pose;
  grab_pose.orientation.w = 0.7071;
  grab_pose.orientation.y = 0.7071;
  //grab_pose.position.z = 1.05;

  // First, we add the object to the world (without using a vector).
  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

  // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
  // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  RCLCPP_INFO(LOGGER, "Attach the object to the robot");
  std::vector<std::string> touch_links;
  touch_links.push_back("rh_p12_rn_l2");
  touch_links.push_back("rh_p12_rn_r2");
  move_group.attachObject(object_to_attach.id, "rh_p12_rn_base", touch_links);

  visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  // Replan, but now with the object in hand.
  move_group.setStartStateToCurrentState();
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // Detaching and Removing Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Now, let's detach the cylinder from the robot's gripper.
  RCLCPP_INFO(LOGGER, "Detach the object from the robot");
  move_group.detachObject(object_to_attach.id);

  // Show text in RViz of status
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object_detached_from_robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

  // Now, let's remove the objects from the world.
  RCLCPP_INFO(LOGGER, "Remove the objects from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Objects_removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

  // END_TUTORIAL
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  return 0;
}