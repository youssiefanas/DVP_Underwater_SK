#pragma once

#include <map>
#include <memory>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "Pose3d.hpp"

namespace frontend {

class KeyFrame;

/**
 * @brief A 3D landmark observed by one or more KeyFrames.
 *
 * Each MapPoint stores its world-frame position, a representative ORB
 * descriptor (the median-distance descriptor across all observers), and the
 * set of KeyFrame observations that see it. MapPoints form the backbone of
 * the local map used for projection-based tracking and the covisibility graph.
 *
 * A MapPoint can be marked as "bad" (e.g. high reprojection error, culled by
 * the smoother) via setBad(), after which it is ignored by tracking and
 * eventually removed from the Map.
 *
 * @note Public data members follow SLAM-prototype convention for fast,
 *       low-ceremony access from the frontend pipeline.
 */
class MapPoint {
public:
  using Ptr = std::shared_ptr<MapPoint>;
  using KeyFramePtr = std::shared_ptr<KeyFrame>;
  using KeyFrameWeakPtr = std::weak_ptr<KeyFrame>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Create a new MapPoint at the given 3D position.
   * @param pos  World-frame coordinates of the landmark.
   * @return Shared pointer to the newly created MapPoint.
   */
  static Ptr create(const Eigen::Vector3d &pos);

  size_t id_;                         ///< Unique sequential MapPoint ID.
  Eigen::Vector3d position_;          ///< 3D position in world frame.
  cv::Mat descriptor_;                ///< Representative ORB descriptor (computed by computeDistinctiveDescriptor()).
  bool isBad_ = false;               ///< True if this point has been invalidated.

  /**
   * @brief Map of KeyFrame observers to their keypoint indices.
   *
   * For each observing KeyFrame, stores the index into that KeyFrame's
   * keypoints/descriptors vectors. This is the core structure for
   * covisibility computation and GTSAM projection factors.
   */
  std::map<KeyFrameWeakPtr, size_t, std::owner_less<KeyFrameWeakPtr>> observations_;

  /**
   * @brief Register a KeyFrame as an observer of this MapPoint.
   * @param kf            The observing KeyFrame (ignored if null).
   * @param keypoint_idx  Index of the keypoint in @p kf that corresponds to this point.
   */
  void addObservation(const KeyFramePtr &kf, size_t keypoint_idx);

  /**
   * @brief Remove a KeyFrame from this point's observer set.
   *
   * If the remaining observation count drops below kMinObservations,
   * the point is automatically marked as bad.
   *
   * @param kf  The KeyFrame to remove.
   */
  void removeObservation(const KeyFramePtr &kf);

  /// @brief Get the number of KeyFrames currently observing this point.
  size_t getObservationCount() const { return observations_.size(); }

  /**
   * @brief Mark this MapPoint as bad and clear all observations.
   *
   * Should be called when reprojection error is too high, the point is
   * an outlier, or when the point falls out of the fixed-lag smoother window.
   */
  void setBad();

  /**
   * @brief Recompute the representative descriptor from all observers.
   *
   * Selects the descriptor whose median Hamming distance to all other
   * observer descriptors is minimized, making it the most "central"
   * and robust to outlier descriptors.
   */
  void computeDistinctiveDescriptor();

private:
  MapPoint(const Eigen::Vector3d &pos, size_t id);
  static size_t next_id_;

  /// Minimum observer count before a point is marked bad on observation removal.
  static constexpr size_t kMinObservations = 2;
};

} // namespace frontend
