#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include <fstream>
#include "frontend/VisualFrontend.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace visual_odom {
class VisualOdomNode : public rclcpp::Node {
public:
    explicit VisualOdomNode(const rclcpp::NodeOptions& options);
    ~VisualOdomNode() = default;

private:
    enum class CameraModel {
      kPinhole,
      kFisheye
    };

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void appendTumPose(const rclcpp::Time& stamp, const frontend::Pose3d& pose);
    void maybeBuildUndistortMaps(const cv::Size& image_size);
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // image_transport::ImageTransport it_;
    image_transport::Subscriber image_subscriber_;
    std::shared_ptr<frontend::VisualFrontend> visual_frontend_;

    // Trajectory Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;

    // Camera model used by frontend and optional image undistortion
    cv::Mat K_;
    cv::Mat dist_coeffs_;
    CameraModel camera_model_{CameraModel::kPinhole};
    bool use_undistort_{false};
    cv::Mat undistort_map1_;
    cv::Mat undistort_map2_;
    cv::Size undistort_map_size_;

    // Optional runtime debug viewer in frontend.
    bool enable_viewer_{false};

    // Optional trajectory logging for GT comparison (TUM RGB-D format).
    bool save_tum_trajectory_{false};
    std::string tum_trajectory_path_;
    std::ofstream tum_trajectory_file_;
};
} // namespace visual_odom
