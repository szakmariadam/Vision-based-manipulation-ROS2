#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class PlanningSceneUpdater : public rclcpp::Node
{
public:
    PlanningSceneUpdater()
        : Node("planning_scene_updater")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/object_detection/classes",
            10,
            std::bind(&PlanningSceneUpdater::topic_callback, this, _1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "'%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanningSceneUpdater>());
    rclcpp::shutdown();
    return 0;
}