#include "frontend/Map.hpp"
#include <algorithm>
#include <iostream>
#include <unordered_set>

namespace frontend {

// ─── KeyFrame management ────────────────────────────────────────
void Map::addKeyFrame(const KeyFrame::Ptr &kf) {
  if (!kf)
    return;
  keyframes_.insert(kf);
}

void Map::removeKeyFrame(const KeyFrame::Ptr &kf) { keyframes_.erase(kf); }

// ─── MapPoint management ────────────────────────────────────────
void Map::addMapPoint(const MapPoint::Ptr &mp) {
  if (!mp)
    return;
  map_points_.insert(mp);
}

void Map::removeMapPoint(const MapPoint::Ptr &mp) { map_points_.erase(mp); }

void Map::cleanBadMapPoints() {
  for (auto it = map_points_.begin(); it != map_points_.end();) {
    if ((*it)->isBad_) {
      it = map_points_.erase(it);
    } else {
      ++it;
    }
  }
}

// ─── Local map for tracking ─────────────────────────────────────
std::vector<MapPoint::Ptr>
Map::getLocalMapPoints(const KeyFrame::Ptr &reference_kf,
                       int n_covisible) const {
  if (!reference_kf)
    return {};

  // 1. Collect "local KeyFrames" = reference + its best covisibles
  std::unordered_set<KeyFrame::Ptr> local_kfs;
  local_kfs.insert(reference_kf);

  auto covisibles = reference_kf->getBestCovisibles(n_covisible);
  for (const auto &kf : covisibles) {
    local_kfs.insert(kf);
  }

  // 2. Collect all MapPoints seen by local KeyFrames (deduplicated)
  std::unordered_set<MapPoint::Ptr> local_mp_set;
  for (const auto &kf : local_kfs) {
    for (const auto &mp : kf->getMapPoints()) {
      if (mp && !mp->isBad_) {
        local_mp_set.insert(mp);
      }
    }
  }

  return std::vector<MapPoint::Ptr>(local_mp_set.begin(), local_mp_set.end());
}

// ─── Fixed-lag smoother window ──────────────────────────────────
std::vector<KeyFrame::Ptr> Map::getKeyFramesInWindow(double current_time,
                                                     double lag_seconds) const {
  std::vector<KeyFrame::Ptr> result;
  double cutoff = current_time - lag_seconds;
  for (const auto &kf : keyframes_) {
    if (kf->getTimestamp() >= cutoff) {
      result.push_back(kf);
    }
  }
  // Sort by timestamp ascending (oldest first)
  std::sort(result.begin(), result.end(),
            [](const KeyFrame::Ptr &a, const KeyFrame::Ptr &b) {
              return a->getTimestamp() < b->getTimestamp();
            });
  return result;
}

} // namespace frontend