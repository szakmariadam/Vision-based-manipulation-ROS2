#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

class MoveToPose : public rclcpp::Node
{
public:
    MoveToPose() : Node("move_to_pose")
    {
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveToPose>());
    rclcpp::shutdown();
    return 0;
}