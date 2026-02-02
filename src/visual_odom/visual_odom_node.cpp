#include "visual_odom_node.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>


namespace visual_odom {


VisualOdomNode::VisualOdomNode(const rclcpp::NodeOptions& options) : rclcpp::Node("VisualOdomNode", options) {
    // Declare parameters with default values
    std::string image_topic_subscriber = this->declare_parameter("image_topic", "cam/image_raw");
    
    frontend::ORBParams params;
    params.n_features = this->declare_parameter("orb.n_features", 1000);
    params.scale_factor = this->declare_parameter("orb.scale_factor", 1.2f);
    params.n_levels = this->declare_parameter("orb.n_levels", 8);
    params.edge_threshold = this->declare_parameter("orb.edge_threshold", 31);
    params.first_level = this->declare_parameter("orb.first_level", 0);
    params.wta_k = this->declare_parameter("orb.wta_k", 2);
    params.score_type = this->declare_parameter("orb.score_type", 0);
    params.patch_size = this->declare_parameter("orb.patch_size", 31);
    params.fast_threshold = this->declare_parameter("orb.fast_threshold", 20);
    params.matcher_type = this->declare_parameter("matcher_type", "NORM_HAMMING");

    // camera parameters
    double fx = this->declare_parameter("Camera.fx", 543.3327734182214);
    double fy = this->declare_parameter("Camera.fy", 542.398772982566);
    double cx = this->declare_parameter("Camera.cx", 489.02536042247897);
    double cy = this->declare_parameter("Camera.cy", 305.38727712002805);

    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    
    visual_frontend_ = std::make_shared<frontend::VisualFrontend>(params, K);
    
    auto it = image_transport::create_subscription(
        this,
        image_topic_subscriber,
        std::bind(&VisualOdomNode::image_callback, this, std::placeholders::_1),
        "raw"
    );
    image_subscriber_ = it;
    RCLCPP_INFO(this->get_logger(), "Visual Odom Node Initialized. Listening on: %s", image_topic_subscriber.c_str());
}

void VisualOdomNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
        // TODO : check if the image is in bgr8 format or grayscale format 
        // get the image format from the yaml file
        
        cv::Mat bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray_image;
        cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // RCLCPP_INFO(this->get_logger(), "Image received. Timestamp: %f", timestamp);
        visual_frontend_->handleImage(gray_image, timestamp);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

}
