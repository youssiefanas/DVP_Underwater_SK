#include "visual_odom_node.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <string>


namespace visual_odom {


VisualOdomNode::VisualOdomNode(const rclcpp::NodeOptions& options) : rclcpp::Node("VisualOdomNode", options) {
    // Declare parameters with default values
    std::string image_topic_subscriber = this->declare_parameter("image_topic", "/camera/rgb/image_color");

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
    enable_viewer_ = this->declare_parameter("frontend.enable_viewer", false);
    const int cv_rng_seed = this->declare_parameter("frontend.cv_rng_seed", 0);
    const int cv_num_threads =
        this->declare_parameter("frontend.cv_num_threads", 1);
    cv::setRNGSeed(cv_rng_seed);
    if (cv_num_threads > 0) {
      cv::setNumThreads(cv_num_threads);
    }

    const int image_queue_size = this->declare_parameter("input.queue_size", 200);
    const bool reliable_qos = this->declare_parameter("input.reliable_qos", true);
    rmw_qos_profile_t image_qos = rmw_qos_profile_default;
    image_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    image_qos.depth = static_cast<size_t>(std::max(1, image_queue_size));
    image_qos.reliability = reliable_qos ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                         : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    double init_scale = this->declare_parameter("mono.init_scale", 1.0);
    if (init_scale <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "mono.init_scale must be > 0. Falling back to 1.0.");
      init_scale = 1.0;
    }

    // camera parameters
    double fx = this->declare_parameter("Camera.fx", 543.3327734182214);
    double fy = this->declare_parameter("Camera.fy", 542.398772982566);
    double cx = this->declare_parameter("Camera.cx", 489.02536042247897);
    double cy = this->declare_parameter("Camera.cy", 305.38727712002805);

    double k1 = this->declare_parameter("Camera.k1", 0.0);
    double k2 = this->declare_parameter("Camera.k2", 0.0);
    double p1 = this->declare_parameter("Camera.p1", 0.0);
    double p2 = this->declare_parameter("Camera.p2", 0.0);
    double k3 = this->declare_parameter("Camera.k3", 0.0);
    use_undistort_ = this->declare_parameter("Camera.undistort", true);

    K_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    dist_coeffs_ = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

    bool has_distortion = std::abs(k1) > 1e-12 || std::abs(k2) > 1e-12 ||
                          std::abs(p1) > 1e-12 || std::abs(p2) > 1e-12 ||
                          std::abs(k3) > 1e-12;
    use_undistort_ = use_undistort_ && has_distortion;

    save_tum_trajectory_ =
        this->declare_parameter("output.save_tum_trajectory", true);
    tum_trajectory_path_ =
        this->declare_parameter("output.tum_trajectory_path",
                                std::string("vo_trajectory_tum.txt"));

    visual_frontend_ =
        std::make_shared<frontend::VisualFrontend>(params, K_, init_scale,
                                                   enable_viewer_);

    if (save_tum_trajectory_) {
      tum_trajectory_file_.open(tum_trajectory_path_, std::ios::out);
      if (!tum_trajectory_file_.is_open()) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not open '%s' for trajectory logging. Disabling "
                    "TUM output.",
                    tum_trajectory_path_.c_str());
        save_tum_trajectory_ = false;
      } else {
        tum_trajectory_file_ << "# timestamp tx ty tz qx qy qz qw\n";
      }
    }

    auto it = image_transport::create_subscription(
        this,
        image_topic_subscriber,
        std::bind(&VisualOdomNode::image_callback, this, std::placeholders::_1),
        "raw",
        image_qos
    );
    image_subscriber_ = it;

    // Initialize Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
    path_msg_.header.frame_id = "map";

    RCLCPP_INFO(
        this->get_logger(),
        "Visual Odom Node Initialized. topic=%s | mono.init_scale=%.3f | "
        "undistort=%s | viewer=%s | tum_log=%s | cv_seed=%d | "
        "cv_threads=%d | queue=%d | qos=%s",
        image_topic_subscriber.c_str(), init_scale,
        use_undistort_ ? "true" : "false",
        enable_viewer_ ? "true" : "false",
        save_tum_trajectory_ ? tum_trajectory_path_.c_str() : "disabled",
        cv_rng_seed, cv_num_threads, image_queue_size,
        reliable_qos ? "reliable" : "best_effort");
}

void VisualOdomNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  try {
    cv::Mat gray_image =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    if (gray_image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image frame.");
      return;
    }

    cv::Mat frontend_image;
    if (use_undistort_) {
      cv::undistort(gray_image, frontend_image, K_, dist_coeffs_);
    } else {
      frontend_image = gray_image;
    }
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    if (visual_frontend_->handleImage(frontend_image, timestamp)) {
      auto frame = visual_frontend_->getLatestFrame();
      if (frame) {
        gtsam::Pose3 pose = frame->getPose();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = "map";

        pose_msg.pose.position.x = pose.x();
        pose_msg.pose.position.y = pose.y();
        pose_msg.pose.position.z = pose.z();

        gtsam::Quaternion q = pose.rotation().toQuaternion();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        path_msg_.header.stamp = msg->header.stamp;
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);
        appendTumPose(msg->header.stamp, pose);
      }
    }

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void VisualOdomNode::appendTumPose(const rclcpp::Time& stamp,
                                   const gtsam::Pose3& pose) {
  if (!save_tum_trajectory_ || !tum_trajectory_file_.is_open()) {
    return;
  }

  const double t = stamp.seconds();
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  tum_trajectory_file_ << std::fixed << std::setprecision(9)
                       << t << " " << pose.x() << " " << pose.y() << " "
                       << pose.z() << " " << q.x() << " " << q.y() << " "
                       << q.z() << " " << q.w() << "\n";
}

}
