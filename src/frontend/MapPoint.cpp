#include "frontend/MapPoint.hpp"

namespace frontend {

std::atomic<long unsigned int> MapPoint::next_id_(0);

MapPoint::MapPoint(const Eigen::Vector3d& pos) : pos_(pos) {
    id_ = next_id_++;
}

Eigen::Vector3d MapPoint::getPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pos_;
}

void MapPoint::setPosition(const Eigen::Vector3d& pos) {
    std::lock_guard<std::mutex> lock(mutex_);
    pos_ = pos;
}

cv::Mat MapPoint::getDescriptor() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return descriptor_.clone();
}

void MapPoint::setDescriptor(const cv::Mat& des) {
    std::lock_guard<std::mutex> lock(mutex_);
    descriptor_ = des.clone();
}

void MapPoint::addObservation(std::shared_ptr<KeyFrame> kf, size_t idx) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Placeholder: In a full system we would store {kf, idx} in a map/list
    // observations_[kf] = idx;
}

void MapPoint::removeObservation(std::shared_ptr<KeyFrame> kf) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Placeholder: observations_.erase(kf);
}

int MapPoint::observationCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    // return observations_.size();
    return 0; // Placeholder
}

bool MapPoint::isBad() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_bad_;
}

void MapPoint::setBadFlag() {
    std::lock_guard<std::mutex> lock(mutex_);
    is_bad_ = true;
}

} // namespace frontend
