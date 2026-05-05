#include "visual_odom_node.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace visual_odom {

// ═══════════════════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════════════════

VisualOdomNode::VisualOdomNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("VisualOdomNode", options) {

  frontend::ORBParams orb_params;
  std::string image_topic;
  double init_scale = 1.0;
  rmw_qos_profile_t image_qos = rmw_qos_profile_default;

  declareAndLoadParameters(orb_params, image_topic, init_scale, image_qos);
  configureCameraModel();

  visual_frontend_ = std::make_shared<frontend::VisualFrontend>(
      orb_params, K_, dist_coeffs_, init_scale, enable_viewer_);

  clahe_enabled_ = this->declare_parameter("clahe.enabled", true);
  const double clahe_clip_limit =
      this->declare_parameter("clahe.clip_limit", 3.0);
  const int clahe_tile_size = this->declare_parameter("clahe.tile_size", 8);
  clahe_ = cv::createCLAHE(clahe_clip_limit,
                            cv::Size(clahe_tile_size, clahe_tile_size));

  openTrajectoryFile();

  const std::string transport =
      this->declare_parameter("input.image_transport", "raw");
  image_subscriber_ = image_transport::create_subscription(
      this, image_topic,
      std::bind(&VisualOdomNode::imageCallback, this, std::placeholders::_1),
      transport, image_qos);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "trajectory", kTrajectoryPubQueueSize);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "odometry", kTrajectoryPubQueueSize);
  path_msg_.header.frame_id = "map";

  // Push extrinsics into the frontend so DVL body-frame velocity is rotated
  // correctly into the camera/world frame during prior assembly.
  visual_frontend_->setBodyCamExtrinsic(T_body_cam_);

  // MIMOSA → VO: subscribe to /graph/odometry (configurable). The same
  // subscription powers both the pose-hint injection (predictPose) and the
  // continuous DVL-velocity-style scale prior — MIMOSA's twist is the body
  // frame velocity from the iSAM2 solution (DVL/IMU dominated; VO contributes
  // only via BetweenFactor on Pose3, not on velocity), so we treat it as a
  // ready-to-use metric reference and avoid coupling dv_slam to a specific
  // DVL driver message (Nortek vs Waterlinked).
  if (mimosa_inject_ || scale_enabled_) {
    const std::string topic_in = this->declare_parameter<std::string>(
        "mimosa.pose_topic_in", "/graph/odometry");
    mimosa_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_in, rclcpp::SensorDataQoS().keep_last(20),
        std::bind(&VisualOdomNode::mimosaOdomCallback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "Subscribed to %s (pose_inject=%d, scale=%d)", topic_in.c_str(),
                static_cast<int>(mimosa_inject_),
                static_cast<int>(scale_enabled_));
  }

  // VO → MIMOSA: publish to /odometry/manager/odometry_in (configurable).
  if (mimosa_publish_) {
    const std::string topic_out = this->declare_parameter<std::string>(
        "mimosa.pose_topic_out", "/visual_odom/odometry_in");
    mimosa_odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(topic_out, 10);
    RCLCPP_INFO(this->get_logger(), "Publishing VO → MIMOSA on %s",
                topic_out.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Visual Odom Node initialized. topic=%s",
              image_topic.c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameter loading
// ═══════════════════════════════════════════════════════════════════════════

void VisualOdomNode::declareAndLoadParameters(
    frontend::ORBParams &orb_params, std::string &image_topic,
    double &init_scale, rmw_qos_profile_t &image_qos) {

  // --- Image topic ---
  image_topic =
      this->declare_parameter("image_topic", "/camera/rgb/image_color");

  // --- ORB feature extractor ---
  orb_params.n_features = this->declare_parameter("orb.n_features", 1000);
  orb_params.scale_factor =
      this->declare_parameter("orb.scale_factor", 1.2f);
  orb_params.n_levels = this->declare_parameter("orb.n_levels", 8);
  orb_params.edge_threshold =
      this->declare_parameter("orb.edge_threshold", 31);
  orb_params.first_level = this->declare_parameter("orb.first_level", 0);
  orb_params.wta_k = this->declare_parameter("orb.wta_k", 2);
  orb_params.score_type = this->declare_parameter("orb.score_type", 0);
  orb_params.patch_size = this->declare_parameter("orb.patch_size", 31);
  orb_params.fast_threshold =
      this->declare_parameter("orb.fast_threshold", 20);
  orb_params.matcher_type =
      this->declare_parameter("matcher_type", "NORM_HAMMING");

  // --- Frontend flags ---
  enable_viewer_ = this->declare_parameter("frontend.enable_viewer", false);
  const int cv_rng_seed = this->declare_parameter("frontend.cv_rng_seed", 0);
  const int cv_num_threads =
      this->declare_parameter("frontend.cv_num_threads", 1);
  cv::setRNGSeed(cv_rng_seed);
  if (cv_num_threads > 0) {
    cv::setNumThreads(cv_num_threads);
  }

  // --- Input QoS ---
  const int queue_size = this->declare_parameter("input.queue_size", 200);
  const bool reliable_qos =
      this->declare_parameter("input.reliable_qos", true);
  image_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  image_qos.depth = static_cast<size_t>(std::max(1, queue_size));
  image_qos.reliability = reliable_qos
                              ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                              : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  // --- Mono initialization scale ---
  init_scale = this->declare_parameter("mono.init_scale", 1.0);
  if (init_scale <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "mono.init_scale must be > 0. Falling back to 1.0.");
    init_scale = 1.0;
  }

  // --- TUM trajectory output ---
  save_tum_trajectory_ =
      this->declare_parameter("output.save_tum_trajectory", true);
  tum_trajectory_path_ = this->declare_parameter(
      "output.tum_trajectory_path", std::string("vo_trajectory_tum.txt"));

  // --- External integration (MIMOSA + DVL) ---
  mimosa_inject_ =
      this->declare_parameter("mimosa.enable_pose_injection", false);
  mimosa_publish_ = this->declare_parameter("mimosa.enable_publish", false);
  mimosa_use_origin_ = this->declare_parameter("mimosa.use_origin", false);
  max_hint_age_s_ = this->declare_parameter("mimosa.max_hint_age_s", 0.25);
  cov_inflate_per_s_ = this->declare_parameter("mimosa.cov_inflate_per_s", 1.0);
  scale_enabled_ = this->declare_parameter("scale.enabled", false);
  dvl_max_var_ = this->declare_parameter("scale.dvl_max_var", 0.01);

  const std::vector<double> default_extrinsic = {0.0, 0.0, 0.0, 0.0,
                                                 0.0, 0.0, 1.0};
  const auto T_body_cam_v = this->declare_parameter<std::vector<double>>(
      "extrinsics.T_body_cam", default_extrinsic);
  T_body_cam_ = parseExtrinsic(T_body_cam_v);
}

// ═══════════════════════════════════════════════════════════════════════════
// Camera configuration
// ═══════════════════════════════════════════════════════════════════════════

void VisualOdomNode::configureCameraModel() {
  const double fx = this->declare_parameter("Camera.fx", 543.3327734182214);
  const double fy = this->declare_parameter("Camera.fy", 542.398772982566);
  const double cx = this->declare_parameter("Camera.cx", 489.02536042247897);
  const double cy = this->declare_parameter("Camera.cy", 305.38727712002805);

  // --- Model selection ---
  std::string model_str =
      this->declare_parameter("Camera.model", "pinhole");
  std::transform(model_str.begin(), model_str.end(), model_str.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });

  if (model_str == "fisheye" || model_str == "equidistant") {
    camera_model_ = CameraModel::kFisheye;
  } else {
    if (model_str != "pinhole") {
      RCLCPP_WARN(this->get_logger(),
                  "Unknown Camera.model='%s'. Falling back to 'pinhole'.",
                  model_str.c_str());
    }
    camera_model_ = CameraModel::kPinhole;
  }

  // --- Distortion coefficients ---
  const double k1 = this->declare_parameter("Camera.k1", 0.0);
  const double k2 = this->declare_parameter("Camera.k2", 0.0);
  const double p1 = this->declare_parameter("Camera.p1", 0.0);
  const double p2 = this->declare_parameter("Camera.p2", 0.0);
  const double k3 = this->declare_parameter("Camera.k3", 0.0);
  const double k4 = this->declare_parameter("Camera.k4", 0.0);
  use_undistort_ = this->declare_parameter("Camera.undistort", true);
  const int image_width = this->declare_parameter("Camera.width", 0);
  const int image_height = this->declare_parameter("Camera.height", 0);

  K_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

  bool has_distortion = false;
  if (camera_model_ == CameraModel::kFisheye) {
    dist_coeffs_ = (cv::Mat_<double>(4, 1) << k1, k2, k3, k4);
    has_distortion =
        std::abs(k1) > kDistortionEpsilon || std::abs(k2) > kDistortionEpsilon ||
        std::abs(k3) > kDistortionEpsilon || std::abs(k4) > kDistortionEpsilon;
    if (std::abs(p1) > kDistortionEpsilon || std::abs(p2) > kDistortionEpsilon) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring Camera.p1/p2 for fisheye model "
                  "(equidistant uses k1..k4 only).");
    }
  } else {
    dist_coeffs_ = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
    has_distortion =
        std::abs(k1) > kDistortionEpsilon || std::abs(k2) > kDistortionEpsilon ||
        std::abs(p1) > kDistortionEpsilon || std::abs(p2) > kDistortionEpsilon ||
        std::abs(k3) > kDistortionEpsilon;
    if (std::abs(k4) > kDistortionEpsilon) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring Camera.k4 for pinhole model.");
    }
  }

  use_undistort_ = use_undistort_ && has_distortion;

  // --- Precompute fisheye remap tables if image size is known ---
  if (use_undistort_ && camera_model_ == CameraModel::kFisheye) {
    if (image_width > 0 && image_height > 0) {
      maybeBuildUndistortMaps(cv::Size(image_width, image_height));
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Camera.width/height not set. Undistortion maps will be "
                  "initialized from the first image frame.");
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// TUM trajectory file
// ═══════════════════════════════════════════════════════════════════════════

void VisualOdomNode::openTrajectoryFile() {
  if (!save_tum_trajectory_) {
    return;
  }

  tum_trajectory_file_.open(tum_trajectory_path_, std::ios::out);
  if (!tum_trajectory_file_.is_open()) {
    RCLCPP_WARN(this->get_logger(),
                "Could not open '%s' for trajectory logging. "
                "Disabling TUM output.",
                tum_trajectory_path_.c_str());
    save_tum_trajectory_ = false;
  } else {
    tum_trajectory_file_ << "# timestamp tx ty tz qx qy qz qw\n";
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Per-frame pipeline
// ═══════════════════════════════════════════════════════════════════════════

void VisualOdomNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
    if (cv_ptr->image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image frame.");
      return;
    }

    const auto preprocess_start = std::chrono::steady_clock::now();
    const cv::Mat frontend_image = preprocessImage(cv_ptr->image);
    const double preprocess_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - preprocess_start)
            .count();

    const double timestamp =
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // 1. Push a MIMOSA-derived pose hint extrapolated to this image's
    //    timestamp; predictPose() consumes it on the next frame.
    if (mimosa_inject_ && latest_mimosa_.valid) {
      frontend::VisualFrontend::ExternalPoseHint hint;
      if (buildHintAt(timestamp, &hint)) {
        visual_frontend_->setExternalPoseHint(hint);
      }
    }

    if (!visual_frontend_->handleImage(frontend_image, timestamp)) {
      return;
    }

    auto frame = visual_frontend_->getLatestFrame();
    if (frame) {
      publishAndLogPose(msg->header.stamp, frame->getPose(),
                        visual_frontend_->getLastCovariance());
    }

    // 2. New-KF event → forward to MIMOSA.
    if (mimosa_publish_) {
      if (auto out = visual_frontend_->consumeBackendOutput()) {
        publishToMimosa(*out);
      }
    }

    const auto &t = visual_frontend_->getLastTiming();
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Frame timing (ms): preprocess=%.1f  extraction=%.1f  "
        "tracking=%.1f  viewer=%.1f  total=%.1f",
        preprocess_ms, t.extraction_ms, t.tracking_ms, t.viewer_ms,
        preprocess_ms + t.total_ms);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

cv::Mat VisualOdomNode::preprocessImage(const cv::Mat &raw) {
  // 1. Convert to grayscale if needed.
  cv::Mat gray;
  if (raw.channels() == 1) {
    gray = raw;
  } else {
    cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);
  }

  // 2. Ensure 8-bit depth (e.g. mono16 → mono8) for ORB compatibility.
  if (gray.depth() != CV_8U) {
    gray.convertTo(gray, CV_8U, 1.0 / 256.0);
  }

  // 3. Optional undistortion.
  cv::Mat undistorted;
  if (use_undistort_ && camera_model_ == CameraModel::kFisheye) {
    maybeBuildUndistortMaps(gray.size());
    cv::remap(gray, undistorted, undistort_map1_, undistort_map2_,
              cv::INTER_LINEAR);
  } else if (use_undistort_ && camera_model_ == CameraModel::kPinhole) {
    cv::undistort(gray, undistorted, K_, dist_coeffs_);
  } else {
    undistorted = gray;
  }

  // 3. CLAHE contrast enhancement (helps feature detection in dark/bright areas).
  if (!clahe_enabled_) {
    return undistorted;
  }
  cv::Mat enhanced;
  clahe_->apply(undistorted, enhanced);
  return enhanced;
}

// ═══════════════════════════════════════════════════════════════════════════
// Pose publishing & logging
// ═══════════════════════════════════════════════════════════════════════════

void VisualOdomNode::publishAndLogPose(
    const builtin_interfaces::msg::Time &stamp,
    const frontend::Pose3d &pose,
    const Eigen::Matrix<double, 6, 6> &covariance) {
  auto odom_msg = toOdometry(stamp, pose, covariance * 1);
  odom_pub_->publish(odom_msg);

  // Path only accepts PoseStamped — extract from odometry
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;
  path_msg_.header.stamp = stamp;
  path_msg_.poses.push_back(pose_stamped);

  // Cap path length to avoid unbounded serialization cost per publish.
  if (path_msg_.poses.size() > kMaxPathPoses) {
    path_msg_.poses.erase(
        path_msg_.poses.begin(),
        path_msg_.poses.begin() +
            static_cast<long>(path_msg_.poses.size() - kMaxPathPoses));
  }
  path_pub_->publish(path_msg_);

  appendTumPose(rclcpp::Time(stamp), pose);
}

nav_msgs::msg::Odometry VisualOdomNode::toOdometry(
    const builtin_interfaces::msg::Time &stamp,
    const frontend::Pose3d &pose,
    const Eigen::Matrix<double, 6, 6> &covariance) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.child_frame_id = "camera";

  const auto &t = pose.translation();
  msg.pose.pose.position.x = t.x();
  msg.pose.pose.position.y = t.y();
  msg.pose.pose.position.z = t.z();

  const Eigen::Quaterniond q(pose.linear());
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // ROS covariance is row-major 6x6: [trans(3), rot(3)]
  // Our covariance is [rot(3), trans(3)] — reorder to ROS convention
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      int ri = i < 3 ? i + 3 : i - 3;
      int rj = j < 3 ? j + 3 : j - 3;
      msg.pose.covariance[ri * 6 + rj] = covariance(i, j);
    }
  }

  return msg;
}

void VisualOdomNode::maybeBuildUndistortMaps(const cv::Size &image_size) {
  if (!use_undistort_ || camera_model_ != CameraModel::kFisheye) {
    return;
  }
  if (image_size.width <= 0 || image_size.height <= 0) {
    return;
  }
  if (!undistort_map1_.empty() && !undistort_map2_.empty() &&
      undistort_map_size_ == image_size) {
    return;
  }

  cv::fisheye::initUndistortRectifyMap(
      K_, dist_coeffs_, cv::Mat::eye(3, 3, CV_64F), K_, image_size, CV_16SC2,
      undistort_map1_, undistort_map2_);
  undistort_map_size_ = image_size;
}

void VisualOdomNode::appendTumPose(const rclcpp::Time &stamp,
                                   const frontend::Pose3d &pose) {
  if (!save_tum_trajectory_ || !tum_trajectory_file_.is_open()) {
    return;
  }

  const double t = stamp.seconds();
  const Eigen::Quaterniond q(pose.linear());
  const auto &tr = pose.translation();
  tum_trajectory_file_ << std::fixed << std::setprecision(9) << t << " "
                        << tr.x() << " " << tr.y() << " " << tr.z() << " "
                        << q.x() << " " << q.y() << " " << q.z() << " "
                        << q.w() << "\n";
}

// ═══════════════════════════════════════════════════════════════════════════
// External integration: MIMOSA pose injection + DVL velocity stream
// ═══════════════════════════════════════════════════════════════════════════

frontend::Pose3d VisualOdomNode::parseExtrinsic(const std::vector<double> &v) {
  frontend::Pose3d T = frontend::Pose3d::Identity();
  if (v.size() != 7) {
    return T;
  }
  const Eigen::Vector3d t(v[0], v[1], v[2]);
  Eigen::Quaterniond q(v[6], v[3], v[4], v[5]); // (w, x, y, z)
  if (q.norm() < 1e-9) {
    return T;
  }
  q.normalize();
  T.linear() = q.toRotationMatrix();
  T.translation() = t;
  return T;
}

void VisualOdomNode::mimosaOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Pose: T_map_body
  const auto &p = msg->pose.pose.position;
  const auto &o = msg->pose.pose.orientation;
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  if (q.norm() < 1e-9) {
    return;
  }
  q.normalize();
  frontend::Pose3d T = frontend::Pose3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(p.x, p.y, p.z);

  latest_mimosa_.T_map_body = T;
  latest_mimosa_.v_body =
      Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z);
  latest_mimosa_.omega_body =
      Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                      msg->twist.twist.angular.z);
  // ROS pose covariance is row-major 6x6 in [trans(3), rot(3)] ordering;
  // un-swap to dv_slam's [rot(3), trans(3)].
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      const int gi = i < 3 ? i + 3 : i - 3;
      const int gj = j < 3 ? j + 3 : j - 3;
      latest_mimosa_.cov(gi, gj) = msg->pose.covariance[i * 6 + j];
    }
  }
  latest_mimosa_.t = rclcpp::Time(msg->header.stamp).seconds();
  latest_mimosa_.valid = true;

  // Forward MIMOSA's body-frame twist to the frontend as a DVL-style
  // velocity sample. ROS twist covariance is row-major 6x6 [linear(3),
  // angular(3)]; the upper-left 3x3 block is what we need.
  if (scale_enabled_ && visual_frontend_) {
    frontend::VisualFrontend::DvlSample s;
    s.t = latest_mimosa_.t;
    s.v_body = latest_mimosa_.v_body;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        s.cov_body(i, j) = msg->twist.covariance[i * 6 + j];
      }
    }
    s.bottom_lock = std::isfinite(s.cov_body.trace()) &&
                    s.cov_body.trace() > 0.0 &&
                    s.cov_body.trace() < dvl_max_var_;
    visual_frontend_->pushDvlSample(s);
  }

  // First-message origin seeding: align dv_slam's world to mimosa_map.
  if (mimosa_use_origin_ && !mimosa_origin_seeded_ && visual_frontend_ &&
      !visual_frontend_->isInitialized()) {
    const frontend::Pose3d T_w_cam = T * T_body_cam_;
    visual_frontend_->setInitialPose(T_w_cam);
    mimosa_origin_seeded_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Seeded VO origin from MIMOSA: t=[%.3f %.3f %.3f]",
                T_w_cam.translation().x(), T_w_cam.translation().y(),
                T_w_cam.translation().z());
  }
}

bool VisualOdomNode::buildHintAt(
    double t_img, frontend::VisualFrontend::ExternalPoseHint *hint) const {
  if (!hint || !latest_mimosa_.valid) {
    return false;
  }
  const double dt = t_img - latest_mimosa_.t;
  if (dt < 0.0 || dt > max_hint_age_s_) {
    return false;
  }

  // Constant body-twist extrapolation: T_map_body_pred = T_map_body *
  // exp([w*dt, v*dt]). We use a first-order SE(3) approximation: rotation via
  // small-angle exp of (omega*dt), translation via (v*dt) in the body frame,
  // then composed.
  const Eigen::Vector3d phi = latest_mimosa_.omega_body * dt;
  const double phi_norm = phi.norm();
  Eigen::Matrix3d dR;
  if (phi_norm < 1e-9) {
    dR = Eigen::Matrix3d::Identity();
  } else {
    dR = Eigen::AngleAxisd(phi_norm, phi / phi_norm).toRotationMatrix();
  }
  frontend::Pose3d delta = frontend::Pose3d::Identity();
  delta.linear() = dR;
  delta.translation() = latest_mimosa_.v_body * dt;

  const frontend::Pose3d T_map_body_pred = latest_mimosa_.T_map_body * delta;
  hint->T_w_cam = T_map_body_pred * T_body_cam_;
  hint->timestamp = t_img;
  hint->covariance =
      latest_mimosa_.cov + cov_inflate_per_s_ * std::abs(dt) *
                               Eigen::Matrix<double, 6, 6>::Identity();
  hint->valid = true;
  return true;
}

void VisualOdomNode::publishToMimosa(const frontend::FrontendOutput &out) {
  if (!mimosa_odom_pub_) {
    return;
  }
  nav_msgs::msg::Odometry msg;
  const int64_t sec = static_cast<int64_t>(std::floor(out.timestamp));
  msg.header.stamp.sec = static_cast<int32_t>(sec);
  msg.header.stamp.nanosec = static_cast<uint32_t>((out.timestamp - sec) * 1e9);
  msg.header.frame_id = "mimosa_map";
  msg.child_frame_id = "mimosa_body";

  // Convert camera-frame world pose to body-frame world pose for MIMOSA.
  // T_w_cam = T_w_body * T_body_cam  ⇒  T_w_body = T_w_cam * T_body_cam⁻¹.
  const frontend::Pose3d T_w_body =
      out.initial_estimate * T_body_cam_.inverse();

  const auto &tr = T_w_body.translation();
  msg.pose.pose.position.x = tr.x();
  msg.pose.pose.position.y = tr.y();
  msg.pose.pose.position.z = tr.z();
  const Eigen::Quaterniond q(T_w_body.linear());
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // [rot(3), trans(3)] → [trans(3), rot(3)] for ROS.
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      const int ri = i < 3 ? i + 3 : i - 3;
      const int rj = j < 3 ? j + 3 : j - 3;
      msg.pose.covariance[ri * 6 + rj] = out.pose_covariance(i, j);
    }
  }
  mimosa_odom_pub_->publish(msg);
}

} // namespace visual_odom
