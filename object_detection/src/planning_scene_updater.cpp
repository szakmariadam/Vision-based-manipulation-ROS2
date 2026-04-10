#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>

// MoveIt
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

using std::placeholders::_1;

class PlanningSceneUpdater : public rclcpp::Node
{
public:
    PlanningSceneUpdater()
        : Node("planning_scene_updater")
    {
        classes_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/object_detection/classes",
            10,
            std::bind(&PlanningSceneUpdater::classes_callback, this, _1)
        );

        obj_pos_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/object_detection/obj_positions",
            10,
            std::bind(&PlanningSceneUpdater::obj_pos_callback, this, _1)
        );

        planning_scene_diff_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
        while (planning_scene_diff_publisher->get_subscription_count() < 1)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PlanningSceneUpdater::process, this));
    }
    void init_robot_model()
    {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this());

        kinematic_model_ = robot_model_loader_->getModel();

        RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model_->getModelFrame().c_str());
    }
private:
    void classes_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        classes_ = msg;
    }

    void obj_pos_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        obj_pos_ = msg;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr classes_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obj_pos_subscription_;

    std_msgs::msg::String::SharedPtr classes_;
    std_msgs::msg::Float32MultiArray::SharedPtr obj_pos_;

    rclcpp::TimerBase::SharedPtr timer_;

    moveit::core::RobotModelPtr kinematic_model_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;

    moveit_msgs::msg::CollisionObject collision_object;
    geometry_msgs::msg::Pose pose;
    shape_msgs::msg::SolidPrimitive primitive;
    moveit_msgs::msg::PlanningScene planning_scene;

    void process()
    {
        if (classes_ && obj_pos_)
        {
            collision_object.header.frame_id = "workspace_link";
            /* The id of the object */
            collision_object.id = "box";

            /* Define a box to be attached */
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = 0.18;
            primitive.dimensions[1] = 0.03;

            pose.position.z = 0.09;
            pose.orientation.w = 1.0;


            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);

            collision_object.operation = collision_object.ADD;

            planning_scene.world.collision_objects.clear();
            planning_scene.world.collision_objects.push_back(collision_object);
            planning_scene.is_diff = true;
            planning_scene_diff_publisher->publish(planning_scene);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for both messages...");
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlanningSceneUpdater>();

    node->init_robot_model();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}