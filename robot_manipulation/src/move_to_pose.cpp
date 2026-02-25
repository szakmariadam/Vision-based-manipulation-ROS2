#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "move_to_pose",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("robot_manipulation");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_arm");

    // Set a target Pose
    auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0;
    msg.orientation.x = 0.707;
    msg.orientation.y = 0.707;
    msg.orientation.z = 0;
    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 1.05;
    return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create collision object for the robot to avoid
    auto const collision_object = [frame_id =
                                    move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "cube";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.05;
        primitive.dimensions[primitive.BOX_Y] = 0.05;
        primitive.dimensions[primitive.BOX_Z] = 0.05;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0;
        box_pose.position.y = 0;
        box_pose.position.z = 1.05;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}