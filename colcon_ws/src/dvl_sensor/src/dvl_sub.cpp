#include "rclcpp/rclcpp.hpp"
#include "dave_interfaces/msg/dvl.hpp"  // Update this with the correct path to your DVL message
enum axis {X, Y, Z};
class DVLSubscriber : public rclcpp::Node
{
public:
    DVLSubscriber() : Node("dvl_subscriber")
    {
        // Update this with the actual topic name
        subscription_ = this->create_subscription<dave_interfaces::msg::DVL>(
            "/dvl/velocity", 10, std::bind(&DVLSubscriber::dvl_callback, this, std::placeholders::_1));
    }
void loop()
{
    // rclcpp::Rate rate(15);
    // while (rclcpp::ok())
    // {
    //     auto now = this->get_clock()->now();
    //     double elapsed_time = now.seconds() - recv_time.seconds();
    //     for(int i = X; i <= Z; i++)
    //     {
    //         displacement[i] += current_velocity[i] * elapsed_time;
    //         RCLCPP_INFO(this->get_logger(), "Displacement - %c: %f", 'X' + i, displacement[i]);
    //     }
    //     recv_time = now;

    //     // Log information
    // }

}

private:
    void dvl_callback(const dave_interfaces::msg::DVL::SharedPtr msg)
    {
        current_velocity[X] = msg->velocity.twist.linear.x;
        current_velocity[Y] = -1*msg->velocity.twist.linear.y;
        current_velocity[Z] = -1*msg->velocity.twist.linear.z;

        auto now = this->get_clock()->now();
        double elapsed_time = now.seconds() - recv_time.seconds();
        for(int i = X; i <= Z; i++)
        {
            displacement[i] += current_velocity[i] * elapsed_time;
            RCLCPP_INFO(this->get_logger(), "Displacement - %c: %f", 'X' + i, displacement[i]);
        }
        recv_time = now;

        // RCLCPP_INFO(this->get_logger(), "DVL Data:");
        // RCLCPP_INFO(this->get_logger(), "Linear Velocities - X: %f, Y: %f, Z: %f",
        //             msg->velocity.twist.linear.x,
        //             msg->velocity.twist.linear.y,
        //             msg->velocity.twist.linear.z);

        // for (const auto& beam : msg->beams) {
        //     if (beam.locked) {
        //         RCLCPP_INFO(this->get_logger(), "Beam %ld locked, Range: %f, Beam Velocities - X: %f, Y: %f, Z: %f",
        //                     beam.id, beam.range,
        //                     beam.velocity.twist.linear.x,
        //                     beam.velocity.twist.linear.y,
        //                     beam.velocity.twist.linear.z);
        //     }
        // }


    }

    double displacement[3] = {0, 0, 0};
    double current_velocity[3] = {0, 0, 0};
    rclcpp::Time recv_time;

    rclcpp::Subscription<dave_interfaces::msg::DVL>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto dvl_subscriber = std::make_shared<DVLSubscriber>();
    // Run the ROS 2 spin function in a separate thread
    std::thread spinner([&]() { rclcpp::spin(dvl_subscriber); });

    // Run your loop function
    // dvl_subscriber->loop();

    // Wait for the spinner thread to finish
    spinner.join();
    rclcpp::shutdown();
    return 0;
}