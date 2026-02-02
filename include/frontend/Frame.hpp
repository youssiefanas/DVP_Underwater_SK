#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <memory>
#include <gtsam/geometry/Pose3.h>
namespace frontend {

class Frame {
public:
    // Typedefs for easy shared pointer usage (Standard in ROS/SLAM)
    using Ptr = std::shared_ptr<Frame>;
    using ConstPtr = std::shared_ptr<const Frame>;

    /**
     * @brief Factory method to create a new Frame.
     * Use this instead of 'new Frame(...)' to ensure ID is auto-incremented safely.
     */
    static Frame::Ptr createFrame(cv::Mat image, double timestamp);

    // Destructor
    ~Frame() = default;

    // --- Setters and Getters ---

    // Set extracted features (Keypoints and Descriptors)
    void setFeatures(const std::vector<cv::KeyPoint>& kps, const cv::Mat& des);

    // Getters
    size_t getId() const { return id_; }
    double getTimestamp() const { return timestamp_; }
    const cv::Mat& getImage() const { return image_; }

    // Pose Getter/Setter
    // Pass by const reference to avoid copying
    const gtsam::Pose3& getPose() const { return pose_; }
    void setPose(const gtsam::Pose3& pose) { pose_ = pose; }
    
    // Access features (const reference to avoid copying)
    const std::vector<cv::KeyPoint>& getKeypoints() const { return keypoints_; }
    const cv::Mat& getDescriptors() const { return descriptors_; }

public:
    // Public Data Members (Optional: Keep public for easy access if you prefer struct-style)
    // In strict OOP, these would be private, but for SLAM prototypes, public is common.
    
    // Pose: Transformation from World to Camera (T_c_w) or vice versa. 
    // Left empty for now, but you will need this later.
    gtsam::Pose3 pose_; 

private:
    // Private constructor: forces users to use the createFrame() factory
    Frame(size_t id, double timestamp, const cv::Mat& image);

    // Data
    size_t id_;
    double timestamp_;
    cv::Mat image_;

    // Features
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
};

} // namespace frontend