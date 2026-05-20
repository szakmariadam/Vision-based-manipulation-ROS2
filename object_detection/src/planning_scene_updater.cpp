#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "object_detection/msg/object_position.hpp"

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

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

using std::placeholders::_1;

class PlanningSceneUpdater : public rclcpp::Node
{
public:
    PlanningSceneUpdater()
        : Node("planning_scene_updater")
    {
        obj_pos_subscription_ = this->create_subscription<object_detection::msg::ObjectPosition>(
            "/object_position",
            10,
            std::bind(&PlanningSceneUpdater::obj_pos_callback, this, _1)
        );

        obj_manip_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/object_manipulation",
            10,
            std::bind(&PlanningSceneUpdater::obj_manip_callback, this, _1)
        );

        manip_status_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/manipulation_status",
            10,
            std::bind(&PlanningSceneUpdater::manip_status_callback, this, _1)
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
    void obj_pos_callback(const object_detection::msg::ObjectPosition::SharedPtr msg)
    {
        obj_pos_ = msg->obj_positions.data;
        

        std::string s;
        s = msg->classes.data.c_str();


        // Remove brackets
        s.erase(std::remove(s.begin(), s.end(), '['), s.end());
        s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
        s.erase(std::remove(s.begin(), s.end(), '\''), s.end());

        // Split by comma
        //std::vector<std::string> classes;
        std::stringstream ss(s);
        std::string item;

        classes.clear();
        while (std::getline(ss, item, ',')) {
            item.erase(0, item.find_first_not_of(" \t"));
            item.erase(item.find_last_not_of(" \t") + 1);

            if (!item.empty()) {
                classes.push_back(item);
            }
        }
    }

    void obj_manip_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        run = false;
    }

    void manip_status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        run = true;
    }

    rclcpp::Subscription<object_detection::msg::ObjectPosition>::SharedPtr obj_pos_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obj_manip_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manip_status_subscription_;

    std::vector<float> obj_pos_;

    rclcpp::TimerBase::SharedPtr timer_;

    moveit::core::RobotModelPtr kinematic_model_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;

    moveit_msgs::msg::CollisionObject remove_object;
    geometry_msgs::msg::Pose pose;
    shape_msgs::msg::SolidPrimitive primitive;

    std::vector<std::string> classes;

    bool run = true;

    void process()
    {
        if (!classes.empty() && run)
        {
            //RCLCPP_INFO(this->get_logger(), "classes size: %i", classes.size());
            for (int i=0; i<classes.size(); i++)
            {
                if(classes[i] == "bottle")
                {
                    moveit_msgs::msg::PlanningScene planning_scene;
                    moveit_msgs::msg::CollisionObject collision_object;

                    collision_object.header.frame_id = "workspace_link";
                    collision_object.id = "bottle";

                    primitive.type = primitive.CYLINDER;
                    primitive.dimensions.resize(2);
                    primitive.dimensions[0] = 0.18;
                    primitive.dimensions[1] = 0.04;

                    pose.position.x = obj_pos_[i*3];
                    pose.position.y = obj_pos_[i*3 + 1];
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
                if(classes[i] == "cup")
                {
                    moveit_msgs::msg::PlanningScene planning_scene;
                    moveit_msgs::msg::CollisionObject collision_object;

                    collision_object.header.frame_id = "workspace_link";
                    collision_object.id = "cup";

                    primitive.type = primitive.CYLINDER;
                    primitive.dimensions.resize(2);
                    primitive.dimensions[0] = 0.12;
                    primitive.dimensions[1] = 0.05;

                    pose.position.x = obj_pos_[i*3];
                    pose.position.y = obj_pos_[i*3 + 1];
                    pose.position.z = 0.06;
                    pose.orientation.w = 1.0;

                    collision_object.primitives.push_back(primitive);
                    collision_object.primitive_poses.push_back(pose);

                    collision_object.operation = collision_object.ADD;

                    planning_scene.world.collision_objects.clear();
                    planning_scene.world.collision_objects.push_back(collision_object);
                    planning_scene.is_diff = true;
                    planning_scene_diff_publisher->publish(planning_scene);
                }
                if(classes[i] == "sports ball")
                {
                    moveit_msgs::msg::PlanningScene planning_scene;
                    moveit_msgs::msg::CollisionObject collision_object;

                    collision_object.header.frame_id = "workspace_link";
                    collision_object.id = "sports ball";

                    primitive.type = primitive.SPHERE;
                    primitive.dimensions.resize(1);
                    primitive.dimensions[0] = 0.05;

                    pose.position.x = obj_pos_[i*3];
                    pose.position.y = obj_pos_[i*3 + 1];
                    pose.position.z = 0.015;
                    pose.orientation.w = 1.0;

                    collision_object.primitives.push_back(primitive);
                    collision_object.primitive_poses.push_back(pose);

                    collision_object.operation = collision_object.ADD;

                    planning_scene.world.collision_objects.clear();
                    planning_scene.world.collision_objects.push_back(collision_object);
                    planning_scene.is_diff = true;
                    planning_scene_diff_publisher->publish(planning_scene);
                }
            }
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