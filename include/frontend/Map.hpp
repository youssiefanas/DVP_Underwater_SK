#pragma once

#include "KeyFrame.hpp"
#include "MapPoint.hpp"
#include <memory>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace frontend {

class Map {
public:
    using Ptr = std::shared_ptr<Map>;

    Map() = default;
    ~Map() = default;

    // --- KeyFrame management ---
    void addKeyFrame(const KeyFrame::Ptr &kf);
    void removeKeyFrame(const KeyFrame::Ptr &kf);

    // --- MapPoint management ---
    void addMapPoint(const MapPoint::Ptr &mp);
    void removeMapPoint(const MapPoint::Ptr &mp);

    /**
     * @brief Remove MapPoints flagged as isBad_ from the global set.
     */
    void cleanBadMapPoints();

    // --- Local map queries (for tracking) ---

    /**
     * @brief Get the set of MapPoints visible from the reference KF
     *        and its N best covisible KeyFrames.
     *        This is the "local map" that the tracker projects into the current
     * frame.
     *
     * @param reference_kf  The most recent KeyFrame
     * @param n_covisible   How many covisible KeyFrames to include
     * @return Deduplicated vector of MapPoints
     */
    std::vector<MapPoint::Ptr>
    getLocalMapPoints(const KeyFrame::Ptr &reference_kf,
                      int n_covisible = 10) const;

    /**
     * @brief Get KeyFrames within a time window (for GTSAM fixed-lag smoother).
     *
     * @param current_time   Current timestamp (seconds)
     * @param lag_seconds    Smoother window width
     * @return KeyFrames with timestamp >= (current_time - lag_seconds)
     */
    std::vector<KeyFrame::Ptr> getKeyFramesInWindow(double current_time,
                                                    double lag_seconds) const;

    // --- Accessors ---
    const std::unordered_set<KeyFrame::Ptr> &getAllKeyFrames() const {
      return keyframes_;
    }
    const std::unordered_set<MapPoint::Ptr> &getAllMapPoints() const {
      return map_points_;
    }
    size_t numKeyFrames() const { return keyframes_.size(); }
    size_t numMapPoints() const { return map_points_.size(); }

  private:
    std::unordered_set<KeyFrame::Ptr> keyframes_;
    std::unordered_set<MapPoint::Ptr> map_points_;
};

} // namespace frontend