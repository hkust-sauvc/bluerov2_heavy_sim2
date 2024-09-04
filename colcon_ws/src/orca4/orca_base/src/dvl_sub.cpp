#include "rclcpp/rclcpp.hpp"
#include "dave_interfaces/msg/dvl.hpp"  // Update this with the correct path to your DVL message

class DVLSubscriber : public rclcpp::Node
{
public:
    DVLSubscriber() : Node("dvl_subscriber")
    {
        // Update this with the actual topic name
        subscription_ = this->create_subscription<dave_interfaces::msg::DVL>(
            "/dvl/velocity", 10, std::bind(&DVLSubscriber::dvl_callback, this, std::placeholders::_1));
    }

private:
    void dvl_callback(const dave_interfaces::msg::DVL::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "DVL Data:");
        RCLCPP_INFO(this->get_logger(), "Linear Velocities - X: %f, Y: %f, Z: %f",
                    msg->velocity.twist.linear.x,
                    msg->velocity.twist.linear.y,
                    msg->velocity.twist.linear.z);

        for (const auto& beam : msg->beams) {
            if (beam.locked) {
                RCLCPP_INFO(this->get_logger(), "Beam %ld locked, Range: %f, Beam Velocities - X: %f, Y: %f, Z: %f",
                            beam.id, beam.range,
                            beam.velocity.twist.linear.x,
                            beam.velocity.twist.linear.y,
                            beam.velocity.twist.linear.z);
            }
        }
    }

    rclcpp::Subscription<dave_interfaces::msg::DVL>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DVLSubscriber>());
    rclcpp::shutdown();
    return 0;
}