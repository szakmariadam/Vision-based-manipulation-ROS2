#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
    TestNode()
    : Node("test_node"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10);

        timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&TestNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Test node started!");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from C++ test node! Count: " + std::to_string(count_++);

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}