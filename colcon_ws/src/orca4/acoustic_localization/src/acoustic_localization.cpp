#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/param_vec.hpp"
#include <math.h>
using std::placeholders::_1;

class AcousticLocalization : public rclcpp::Node
{
public:
  AcousticLocalization(): Node("acoustic_localization")
  {
    // Create a publisher
    acoustic_pinger_subscriber = this->create_subscription<ros_gz_interfaces::msg::ParamVec>(
      "/bluerov2_heavy/sensors/acoustics/receiver/range_bearing", 10, std::bind(&AcousticLocalization::acoustic_pinger_callback, this, _1));
  }
private:
    void acoustic_pinger_callback(const ros_gz_interfaces::msg::ParamVec::SharedPtr msg)
    {
        elevation = msg->params[0].value.double_value;
        bearing = msg->params[1].value.double_value;
        range = msg->params[2].value.double_value;
        xyz[0] = -range * cos(elevation) * cos(bearing);
        xyz[1] = -range * cos(elevation) * sin(bearing);
        xyz[2] = -range * sin(elevation);
        std::cout << "x: " << xyz[0] << " y: " << xyz[1] << " z: " << xyz[2] << std::endl;
    }
    rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr acoustic_pinger_subscriber;
    double range;
    double bearing;
    double elevation;
    double xyz[3];
};
int main()
{
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<AcousticLocalization>());
    rclcpp::shutdown();
    return 0;
}