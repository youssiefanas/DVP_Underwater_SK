#pragma once

#include <map>
#include <memory>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "Pose3d.hpp"

namespace frontend {

// Forward declarations — no circular includes
class KeyFrame;

class MapPoint {
public:
  using Ptr = std::shared_ptr<MapPoint>;
  using KeyFramePtr = std::shared_ptr<KeyFrame>;

  /**
   * @brief Factory method.
   */
  static Ptr create(const Eigen::Vector3d &pos);

  // --- Public data (SLAM-prototype style) ---
  size_t id_;
  Eigen::Vector3d position_;
  cv::Mat descriptor_; // Representative descriptor (e.g. median of observers)
                       // calculated by MapPoint::computeDistinctiveDescriptor()
  bool isBad_ = false;

  /**
   * @brief Observations: which KeyFrames see this point and at which keypoint
   * index. This is the core data structure for covisibility and GTSAM
   * projection factors.
   */
  std::map<KeyFramePtr, size_t> observations_;

  // --- Observation management ---
  void addObservation(const KeyFramePtr &kf, size_t keypoint_idx);
  void removeObservation(const KeyFramePtr &kf);
  size_t getObservationCount() const { return observations_.size(); }

  /**
   * @brief Mark this point as bad.
   *        Should be called when reprojection error is too high or
   *        when the point falls out of the fixed-lag smoother window.
   */
  void setBad();

  /**
   * @brief Compute the best representative descriptor from all observers.
   *        Picks the descriptor with minimum median Hamming distance to all
   * others.
   */
  void computeDistinctiveDescriptor();

private:
  MapPoint(const Eigen::Vector3d &pos, size_t id);
  static size_t next_id_;

  static constexpr size_t kMinObservations = 2;
};

} // namespace frontend