#include "frontend/MapPoint.hpp"

namespace frontend {
    
MapPoint::MapPoint(const Eigen::Vector3d& pos) : position_(pos) {}

MapPoint::Ptr MapPoint::create(const Eigen::Vector3d& pos) {
    return Ptr(new MapPoint(pos));
}

void MapPoint::addObservation(const FramePtr& frame, size_t keypoint_idx) {
    if (!frame) {
        return;
    }
    observations_[frame] = keypoint_idx;
}

} // namespace frontend

