#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <opencv2/core.hpp>
#include <gtsam/geometry/Pose3.h>
#include "MapPoint.hpp"
#include "Frame.hpp"

namespace frontend {

class KeyFrame {
public:
    using Ptr = std::shared_ptr<KeyFrame>;

    /**
     * @brief Create a KeyFrame from a Frame.
     * Takes ownership of features and pose from the Frame.
     */
    KeyFrame(Frame::Ptr frame);
    ~KeyFrame() = default;

    // --- Getters ---
    size_t getId() const { return id_; }
    double getTimestamp() const { return timestamp_; }
    gtsam::Pose3 getPose() const;
    void setPose(const gtsam::Pose3& pose);

    // Features
    const std::vector<cv::KeyPoint>& getKeypoints() const { return keypoints_; }
    const cv::Mat& getDescriptors() const { return descriptors_; }
    
    // MapPoints
    // Returns the MapPoint observed at feature index 'idx', or nullptr
    MapPoint::Ptr getMapPoint(size_t idx) const;
    void addMapPoint(MapPoint::Ptr mp, size_t idx);

public:
    long unsigned int id_;
    static std::atomic<long unsigned int> next_id_;

private:
    double timestamp_;
    gtsam::Pose3 pose_;
    
    // Features (Static for KeyFrame)
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    // MapPoints observed by this KeyFrame (aligned with keypoints_)
    std::vector<MapPoint::Ptr> map_points_;
    
    mutable std::mutex mutex_;
};

} // namespace frontend
