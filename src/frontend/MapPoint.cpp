#include "frontend/MapPoint.hpp"

#include <algorithm>
#include <climits>
#include <vector>

#include "frontend/KeyFrame.hpp"

namespace frontend {

size_t MapPoint::next_id_ = 0;

MapPoint::MapPoint(const Eigen::Vector3d &pos, size_t id)
    : id_(id), position_(pos) {}

MapPoint::Ptr MapPoint::create(const Eigen::Vector3d &pos) {
  Ptr mp(new MapPoint(pos, next_id_++));
  return mp;
}

void MapPoint::addObservation(const KeyFramePtr &kf, size_t keypoint_idx) {
  if (!kf)
    return;
  observations_[std::weak_ptr<KeyFrame>(kf)] = keypoint_idx;
}

void MapPoint::removeObservation(const KeyFramePtr &kf) {
  if (observations_.erase(kf) > 0) {
    if (observations_.size() < kMinObservations) {
      isBad_ = true;
    }
  }
}

void MapPoint::setBad() {
  isBad_ = true;
  observations_.clear();
}

void MapPoint::computeDistinctiveDescriptor() {
  if (observations_.empty())
    return;

  // Collect all descriptors from observing KeyFrames
  std::vector<cv::Mat> descriptors;
  descriptors.reserve(observations_.size());

  for (const auto &[kf, idx] : observations_) {
    if (idx < static_cast<size_t>(kf->getDescriptors().rows)) {
      descriptors.push_back(kf->getDescriptors().row(static_cast<int>(idx)));
    }
    ++it;
  }

  if (descriptors.empty())
    return;
  if (descriptors.size() == 1) {
    descriptor_ = descriptors[0].clone();
    return;
  }

  // Pick the descriptor with minimum median Hamming distance to all others
  // (most "central" descriptor, robust to outliers)
  const size_t N = descriptors.size();
  std::vector<std::vector<int>> distances(N, std::vector<int>(N, 0));
  for (size_t i = 0; i < N; i++) {
    for (size_t j = i + 1; j < N; j++) {
      int d = cv::norm(descriptors[i], descriptors[j], cv::NORM_HAMMING);
      distances[i][j] = d;
      distances[j][i] = d;
    }
  }

  int best_median = INT_MAX;
  size_t best_idx = 0;
  for (size_t i = 0; i < N; i++) {
    std::vector<int> dists_i = distances[i];
    std::sort(dists_i.begin(), dists_i.end());
    int median = dists_i[N / 2];
    if (median < best_median) {
      best_median = median;
      best_idx = i;
    }
  }

  descriptor_ = descriptors[best_idx].clone();
}

} // namespace frontend
