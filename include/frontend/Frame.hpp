#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "MapPoint.hpp"
#include "Pose3d.hpp"

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
    static Frame::Ptr createFrame(const cv::Mat& image, double timestamp);

    // Destructor
    ~Frame() = default;

    // --- Setters and Getters ---

    // Set extracted features (Keypoints and Descriptors)
    void setFeatures(const std::vector<cv::KeyPoint>& kps, const cv::Mat& des);

    
    size_t getId() const;
    double getTimestamp() const;
    const cv::Mat& getImage() const;

    // Pose Getter/Setter
    const Pose3d& getPose() const;
    void setPose(const Pose3d& pose);

    // Feature access
    const std::vector<cv::KeyPoint>& getKeypoints() const;
    const cv::Mat& getDescriptors() const;

    // MapPoint access: one-to-one with keypoints_
    const std::vector<MapPoint::Ptr>& getMapPoints() const;
    std::vector<MapPoint::Ptr>& accessMapPoints(); // non-const access
    void ensureMapPointVectorSized(size_t n); // keep invariant

private:
    Frame(size_t id, double timestamp, const cv::Mat& image);

    size_t id_;
    double timestamp_;
    cv::Mat image_;
    Pose3d pose_{Pose3d::Identity()};

    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    std::vector<MapPoint::Ptr> map_points_;

    static size_t next_id_;
};

} // namespace frontend
