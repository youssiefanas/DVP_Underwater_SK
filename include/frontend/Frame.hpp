#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "MapPoint.hpp"
#include "Pose3d.hpp"

namespace frontend {

/**
 * @brief A single image observation with extracted features and associated map data.
 *
 * Frames are the fundamental unit of visual input in the SLAM frontend.
 * Each frame stores a grayscale image, its detected keypoints and descriptors,
 * a camera pose (T_world_camera), and a vector of MapPoint associations that
 * is kept in 1:1 correspondence with the keypoints vector.
 *
 * Frames are created exclusively through the createFrame() factory method,
 * which assigns a monotonically increasing ID.
 */
class Frame {
public:
    using Ptr = std::shared_ptr<Frame>;
    using ConstPtr = std::shared_ptr<const Frame>;

    /**
     * @brief Factory method to create a new Frame with an auto-incremented ID.
     * @param image    Grayscale input image (will be deep-copied internally).
     * @param timestamp  Capture time of the image, in seconds.
     * @return Shared pointer to the newly created Frame.
     */
    static Frame::Ptr createFrame(const cv::Mat& image, double timestamp);

    ~Frame() = default;

    /**
     * @brief Store extracted keypoints and descriptors for this frame.
     *
     * Resets the MapPoint association vector to the same size as @p kps,
     * filled with nullptrs, to maintain the 1:1 invariant.
     *
     * @param kps  Detected keypoints.
     * @param des  Corresponding descriptor matrix (one row per keypoint).
     */
    void setFeatures(const std::vector<cv::KeyPoint>& kps, const cv::Mat& des);

    /// @brief Get the unique sequential ID of this frame.
    size_t getId() const;

    /// @brief Get the capture timestamp in seconds.
    double getTimestamp() const;

    /// @brief Get the stored grayscale image.
    const cv::Mat& getImage() const;

    /// @brief Get the current world-to-camera pose (T_world_camera).
    const Pose3d& getPose() const;

    /**
     * @brief Set the camera pose in world coordinates.
     * @param pose  The SE(3) transform T_world_camera.
     */
    void setPose(const Pose3d& pose);

    /**
     * @brief Set an IMU-derived attitude prior for this frame's pose.
     * @param R_world_camera  Camera-frame rotation reconstructed from IMU
     *                        (R_world_imu * R_imu_camera).
     */
    void setAttitudePrior(const Eigen::Matrix3d& R_world_camera);

    /// @brief Get the attitude prior, if one was set; nullopt otherwise.
    const std::optional<Eigen::Matrix3d>& getAttitudePrior() const;

    /// @brief Get the detected keypoints (read-only).
    const std::vector<cv::KeyPoint>& getKeypoints() const;

    /// @brief Get the descriptor matrix (read-only).
    const cv::Mat& getDescriptors() const;

    /// @brief Get the MapPoint associations, one per keypoint (read-only).
    const std::vector<MapPoint::Ptr>& getMapPoints() const;

    /// @brief Get mutable access to the MapPoint associations vector.
    std::vector<MapPoint::Ptr>& accessMapPoints();

    /**
     * @brief Ensure the MapPoint vector is at least @p n elements long.
     *
     * Pads with nullptr if the current size is smaller than @p n.
     * Used to maintain the 1:1 invariant when keypoints are added externally.
     *
     * @param n  Minimum required size.
     */
    void ensureMapPointVectorSized(size_t n);

private:
    Frame(size_t id, double timestamp, const cv::Mat& image);

    size_t id_;
    double timestamp_;
    cv::Mat image_;
    Pose3d pose_{Pose3d::Identity()};

    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    std::vector<MapPoint::Ptr> map_points_;

    std::optional<Eigen::Matrix3d> attitude_prior_R_world_camera_;

    static size_t next_id_;
};

} // namespace frontend
