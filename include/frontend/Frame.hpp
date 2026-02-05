#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <memory>
#include <gtsam/geometry/Pose3.h>
#include "MapPoint.hpp"

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

    
    size_t getId() const;
    double getTimestamp() const;
    const cv::Mat& getImage() const;

    // Pose Getter/Setter
    const gtsam::Pose3& getPose() const;
    void setPose(const gtsam::Pose3& pose);

    // Feature access
    const std::vector<cv::KeyPoint>& getKeypoints() const;
    const cv::Mat& getDescriptors() const;

    // MapPoint access: one-to-one with keypoints_
    const std::vector<MapPoint::Ptr>& getMapPoints() const;
    std::vector<MapPoint::Ptr>& accessMapPoints(); // non-const access
    void ensureMapPointVectorSized(size_t n); // keep invariant

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

    // MapPoints
    std::vector<MapPoint::Ptr> map_points_;
};

} // namespace frontend