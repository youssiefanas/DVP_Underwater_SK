#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/core.hpp>

#include "KeyFrame.hpp"
#include "MapPoint.hpp"

namespace frontend {

/**
 * @brief Global container for all KeyFrames and MapPoints in the SLAM system.
 *
 * The Map owns the canonical sets of KeyFrames and MapPoints and provides:
 *   - Insertion / removal of KeyFrames and MapPoints.
 *   - Garbage collection of MapPoints marked as bad.
 *   - Local-map queries for projection-based tracking (reference KF +
 *     its N best covisible KeyFrames).
 *   - Time-windowed KeyFrame queries for the GTSAM fixed-lag smoother.
 */
class Map {
public:
    using Ptr = std::shared_ptr<Map>;

    Map() = default;
    ~Map() = default;

    /**
     * @brief Insert a KeyFrame into the global set.
     * @param kf  KeyFrame to add (no-op if null).
     */
    void addKeyFrame(const KeyFrame::Ptr &kf);

    /**
     * @brief Remove a KeyFrame from the global set.
     * @param kf  KeyFrame to remove.
     */
    void removeKeyFrame(const KeyFrame::Ptr &kf);

    /**
     * @brief Insert a MapPoint into the global set.
     * @param mp  MapPoint to add (no-op if null).
     */
    void addMapPoint(const MapPoint::Ptr &mp);

    /**
     * @brief Remove a MapPoint from the global set.
     * @param mp  MapPoint to remove.
     */
    void removeMapPoint(const MapPoint::Ptr &mp);

    /**
     * @brief Remove all MapPoints whose isBad_ flag is set.
     *
     * Should be called periodically (e.g. after KeyFrame insertion) to
     * keep the global set clean.
     */
    void cleanBadMapPoints();

    /**
     * @brief Build the local map visible from a reference KeyFrame.
     *
     * Collects all MapPoints observed by the reference KeyFrame and its
     * @p n_covisible best covisible KeyFrames. The result is deduplicated.
     *
     * @param reference_kf  The anchor KeyFrame for the local map.
     * @param n_covisible   Maximum number of covisible KeyFrames to include.
     * @return Deduplicated vector of valid (non-bad) MapPoints.
     */
    std::vector<MapPoint::Ptr>
    getLocalMapPoints(const KeyFrame::Ptr &reference_kf,
                      int n_covisible = 10) const;

    /**
     * @brief Get KeyFrames within a time window (for the GTSAM fixed-lag smoother).
     *
     * Returns all KeyFrames whose timestamp falls within
     * [current_time - lag_seconds, current_time], sorted ascending by timestamp.
     *
     * @param current_time  Current wall-clock or bag time (seconds).
     * @param lag_seconds   Width of the smoother window (seconds).
     * @return KeyFrames in the window, sorted oldest-first.
     */
    std::vector<KeyFrame::Ptr> getKeyFramesInWindow(double current_time,
                                                    double lag_seconds) const;

    /// @brief Get the full set of KeyFrames (read-only).
    const std::unordered_set<KeyFrame::Ptr> &getAllKeyFrames() const {
      return keyframes_;
    }

    /// @brief Get the full set of MapPoints (read-only).
    const std::unordered_set<MapPoint::Ptr> &getAllMapPoints() const {
      return map_points_;
    }

    /// @brief Get the number of KeyFrames currently in the map.
    size_t numKeyFrames() const { return keyframes_.size(); }

    /// @brief Get the number of MapPoints currently in the map.
    size_t numMapPoints() const { return map_points_.size(); }

  private:
    std::unordered_set<KeyFrame::Ptr> keyframes_;   ///< All active KeyFrames.
    std::unordered_set<MapPoint::Ptr> map_points_;  ///< All active MapPoints.
};

} // namespace frontend
