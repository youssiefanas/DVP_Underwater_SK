#pragma once

#include <fstream>

#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "frontend/VisualFrontend.hpp"

namespace visual_odom {

/**
 * @brief ROS 2 node that runs monocular visual odometry.
 *
 * Subscribes to a camera image topic, preprocesses each frame (grayscale
 * conversion, optional undistortion, CLAHE contrast enhancement), feeds it
 * to the VisualFrontend, and publishes the estimated trajectory as a
 * nav_msgs::Path. Optionally logs poses in TUM RGB-D format for ground-truth
 * comparison.
 *
 * All tuneable parameters (camera intrinsics, ORB settings, QoS, etc.) are
 * declared as ROS 2 parameters and can be set via a YAML config file.
 */
class VisualOdomNode : public rclcpp::Node {
public:
    explicit VisualOdomNode(const rclcpp::NodeOptions& options);
    ~VisualOdomNode() = default;

private:
    /// @brief Supported camera distortion models.
    enum class CameraModel {
      kPinhole,   ///< Standard pinhole (radial-tangential distortion).
      kFisheye    ///< Equidistant / fisheye (k1–k4 distortion).
    };

    // ── Initialization helpers (called once from the constructor) ──────────

    /**
     * @brief Declare and load all ROS 2 parameters, returning the parsed config.
     *
     * Populates ORB params, camera intrinsics, QoS settings, and output flags.
     */
    void declareAndLoadParameters(frontend::ORBParams &orb_params,
                                  std::string &image_topic,
                                  double &init_scale,
                                  rmw_qos_profile_t &image_qos);

    /**
     * @brief Configure camera intrinsics, distortion, and undistortion maps.
     *
     * Reads the declared camera parameters, builds K_ and dist_coeffs_,
     * and optionally precomputes remap tables for fisheye undistortion.
     */
    void configureCameraModel();

    /**
     * @brief Open the TUM trajectory file if trajectory saving is enabled.
     */
    void openTrajectoryFile();

    // ── Per-frame pipeline ────────────────────────────────────────────────

    /**
     * @brief Image subscription callback — the main per-frame entry point.
     * @param msg  Incoming camera image.
     */
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    /**
     * @brief Convert a raw (possibly colour) image to a preprocessed grayscale
     *        frame ready for the frontend.
     *
     * Applies grayscale conversion, optional undistortion, and CLAHE contrast
     * enhancement.
     *
     * @param raw  Input image (grayscale or BGR).
     * @return Preprocessed grayscale image.
     */
    cv::Mat preprocessImage(const cv::Mat &raw);

    /**
     * @brief Publish the current pose on the trajectory topic and log to TUM file.
     * @param stamp       ROS timestamp of the originating image.
     * @param pose        World-frame camera pose to publish.
     * @param covariance  6x6 pose covariance [rot(3), trans(3)].
     */
    void publishAndLogPose(const builtin_interfaces::msg::Time &stamp,
                           const frontend::Pose3d &pose,
                           const Eigen::Matrix<double, 6, 6> &covariance);

    /**
     * @brief Build a nav_msgs::Odometry from an SE(3) pose and covariance.
     * @param stamp       ROS timestamp.
     * @param pose        SE(3) camera pose (T_world_camera).
     * @param covariance  6x6 covariance [rot(3), trans(3)].
     * @return Populated Odometry message.
     */
    static nav_msgs::msg::Odometry toOdometry(
        const builtin_interfaces::msg::Time &stamp,
        const frontend::Pose3d &pose,
        const Eigen::Matrix<double, 6, 6> &covariance);

    /**
     * @brief Lazily build fisheye undistortion remap tables.
     *
     * No-op if maps are already built for the given size, or if the camera
     * model is not fisheye, or if undistortion is disabled.
     *
     * @param image_size  Resolution of the input images.
     */
    void maybeBuildUndistortMaps(const cv::Size &image_size);

    /**
     * @brief Append a single pose line to the TUM trajectory file.
     * @param stamp  ROS timestamp.
     * @param pose   SE(3) camera pose.
     */
    void appendTumPose(const rclcpp::Time &stamp,
                       const frontend::Pose3d &pose);

    // ── Members ───────────────────────────────────────────────────────────

    /// Image transport subscriber (supports compressed topics).
    image_transport::Subscriber image_subscriber_;

    /// Core visual SLAM frontend.
    std::shared_ptr<frontend::VisualFrontend> visual_frontend_;

    /// Trajectory publisher (nav_msgs::Path).
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    /// Odometry publisher (pose with covariance).
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    /// Accumulated path message (grows with each tracked frame).
    nav_msgs::msg::Path path_msg_;

    /// CLAHE histogram equalizer (created once, reused every frame).
    cv::Ptr<cv::CLAHE> clahe_;

    // ── Camera model ──────────────────────────────────────────────────────

    cv::Mat K_;                                       ///< 3×3 camera intrinsic matrix.
    cv::Mat dist_coeffs_;                             ///< Distortion coefficients.
    CameraModel camera_model_{CameraModel::kPinhole}; ///< Active distortion model.
    bool use_undistort_{false};                        ///< True if runtime undistortion is active.
    cv::Mat undistort_map1_;                           ///< Remap table X (fisheye only).
    cv::Mat undistort_map2_;                           ///< Remap table Y (fisheye only).
    cv::Size undistort_map_size_;                      ///< Resolution of the cached remap tables.

    // ── Feature flags ─────────────────────────────────────────────────────

    bool enable_viewer_{false};          ///< Show live debug viewer window.

    // ── TUM trajectory logging ────────────────────────────────────────────

    bool save_tum_trajectory_{false};    ///< Whether TUM trajectory output is enabled.
    std::string tum_trajectory_path_;    ///< Output file path.
    std::ofstream tum_trajectory_file_;  ///< Open file stream (valid only when enabled).

    // ── Named constants ───────────────────────────────────────────────────

    static constexpr double kClaheClipLimit = 3.0;         ///< CLAHE contrast clip limit.
    static constexpr int kClaheTileSize = 8;               ///< CLAHE tile grid dimension.
    static constexpr double kDistortionEpsilon = 1e-12;    ///< Threshold to detect non-zero distortion.
    static constexpr int kTrajectoryPubQueueSize = 10;     ///< Publisher queue depth for the path topic.
};

} // namespace visual_odom
