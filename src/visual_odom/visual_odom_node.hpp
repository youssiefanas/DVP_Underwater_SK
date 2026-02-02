#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "frontend/VisualFrontend.hpp" 


namespace visual_odom {
class VisualOdomNode : public rclcpp::Node {
public:
    explicit VisualOdomNode(const rclcpp::NodeOptions& options);
    ~VisualOdomNode() = default;

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // image_transport::ImageTransport it_;
    image_transport::Subscriber image_subscriber_;
    std::shared_ptr<frontend::VisualFrontend> visual_frontend_;
};
} // namespace visual_odom
