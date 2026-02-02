#include "frontend/KeyFrame.hpp"

namespace frontend {

std::atomic<long unsigned int> KeyFrame::next_id_(0);

KeyFrame::KeyFrame(Frame::Ptr frame) 
    : timestamp_(frame->getTimestamp()), 
      pose_(frame->getPose()),
      keypoints_(frame->getKeypoints()),
      descriptors_(frame->getDescriptors()) 
{
    id_ = next_id_++;
    
    // Initialize map_points_ with nullptrs
    map_points_.resize(keypoints_.size(), nullptr);
    
    // If we were tracking, some map points might be known from the Frame
    // (This requires Frame to have MapPoint references, which we will add next)
    // For now, we manually transfer if needed or assume connections are built later.
}

gtsam::Pose3 KeyFrame::getPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
}

void KeyFrame::setPose(const gtsam::Pose3& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = pose;
}

MapPoint::Ptr KeyFrame::getMapPoint(size_t idx) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (idx >= map_points_.size()) return nullptr;
    return map_points_[idx];
}

void KeyFrame::addMapPoint(MapPoint::Ptr mp, size_t idx) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (idx < map_points_.size()) {
        map_points_[idx] = mp;
    }
}

} // namespace frontend
