#include "frontend/KeyFrame.hpp"

#include <algorithm>
#include <iostream>

#include "frontend/Frame.hpp"

namespace frontend {

// Static counter
size_t KeyFrame::next_id_ = 0;

KeyFrame::KeyFrame(std::shared_ptr<Frame> frame, size_t id)
    : id_(id),
      frame_id_(frame->getId()),
      timestamp_(frame->getTimestamp()),
      pose_(frame->getPose()),
      image_(frame->getImage()),           // shallow copy (cv::Mat ref-counted)
      keypoints_(frame->getKeypoints()),
      descriptors_(frame->getDescriptors()) // shallow copy
{
    pose_key_ = gtsam::Symbol('x', id_);

    // Copy MapPoint associations from the frame
    const auto& frame_mps = frame->getMapPoints();
    map_points_.resize(keypoints_.size(), nullptr);
    for (size_t i = 0; i < frame_mps.size() && i < map_points_.size(); i++) {
        map_points_[i] = frame_mps[i];
    }
}

KeyFrame::Ptr KeyFrame::create(std::shared_ptr<Frame> frame) {
    if (!frame) return nullptr;
    Ptr kf(new KeyFrame(frame, next_id_));
    next_id_++;
    return kf;
}

size_t KeyFrame::countMapPoints() const {
    size_t count = 0;
    for (const auto& mp : map_points_) {
        if (mp && !mp->isBad_) count++;
    }
    return count;
}

void KeyFrame::addCovisibleKeyFrame(KeyFrame::Ptr kf, int weight) {
    if (!kf || kf.get() == this) return;
    covisibility_weights_[kf] = weight;
}

void KeyFrame::updateCovisibility() {
    // Count shared MapPoints with every other KeyFrame
    std::map<KeyFrame::Ptr, int> kf_counter;

    for (size_t i = 0; i < map_points_.size(); i++) {
        MapPoint::Ptr mp = map_points_[i];
        if (!mp || mp->isBad_) continue;

        for (const auto& [obs_kf, obs_idx] : mp->observations_) {
            if (obs_kf.get() == this) continue;   // skip self
            kf_counter[obs_kf]++;
        }
    }

    // Update internal structures
    covisibility_weights_.clear();
    ordered_covisibles_.clear();

    for (const auto& [kf, weight] : kf_counter) {
        if (weight < 5) continue;  // Ignore weak connections (threshold tunable)
        covisibility_weights_[kf] = weight;
        ordered_covisibles_.emplace_back(weight, kf);
    }

    // Sort descending by weight
    std::sort(ordered_covisibles_.begin(), ordered_covisibles_.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
}

std::vector<KeyFrame::Ptr> KeyFrame::getBestCovisibles(int n) const {
    std::vector<KeyFrame::Ptr> result;
    int count = 0;
    for (const auto& [weight, kf] : ordered_covisibles_) {
        result.push_back(kf);
        if (++count >= n) break;
    }
    return result;
}

std::vector<KeyFrame::Ptr> KeyFrame::getAllCovisibles() const {
    std::vector<KeyFrame::Ptr> result;
    for (const auto& [weight, kf] : ordered_covisibles_) {
        result.push_back(kf);
    }
    return result;
}
void KeyFrame::syncMapPointsToFrame(std::shared_ptr<Frame> frame) const {
    if (!frame) return;
    // Make sure the vector is large enough
    frame->ensureMapPointVectorSized(frame->getKeypoints().size());

    size_t limit = std::min(map_points_.size(), frame->getMapPoints().size());
    for (size_t i = 0; i < limit; i++) {
      // Overwrite even if frame already has something at this index,
      // because the KeyFrame's version is authoritative after triangulation
      if (map_points_[i] && !map_points_[i]->isBad_) {
        frame->accessMapPoints()[i] = map_points_[i];
      }
    }
}

} // namespace frontend