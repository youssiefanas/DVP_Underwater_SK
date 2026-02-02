#include "rclcpp/rclcpp.hpp"
#include "visual_odom_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions();
    auto visual_odom_node = std::make_shared<visual_odom::VisualOdomNode>(options);
    rclcpp::spin(visual_odom_node);
    rclcpp::shutdown();

    return 0;
}

