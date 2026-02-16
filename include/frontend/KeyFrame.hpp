#pragma once

#include "MapPoint.hpp"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <memory>
#include <opencv2/core.hpp>
#include <set>
#include <vector>

namespace frontend {

// Forward declare Frame so we don't create a circular dependency
class Frame;

class KeyFrame {
public:
  using Ptr = std::shared_ptr<KeyFrame>;

  /**
   * @brief Factory: promote a Frame into a KeyFrame.
   *        Copies features, descriptors, pose, and image from the frame.
   *        Assigns a monotonically increasing KeyFrame ID.
   */
  static Ptr create(std::shared_ptr<Frame> frame);

  // --- Identifiers ---
  size_t getId() const { return id_; }
  size_t getFrameId() const { return frame_id_; }
  double getTimestamp() const { return timestamp_; }

  // --- GTSAM integration ---
  gtsam::Key getPoseKey() const { return pose_key_; } // gtsam::Symbol('x', id_)

  // --- Pose (T_world_camera) ---
  const gtsam::Pose3 &getPose() const { return pose_; }
  void setPose(const gtsam::Pose3 &pose) { pose_ = pose; }

  // --- Features (copied from Frame at creation time) ---
  const std::vector<cv::KeyPoint> &getKeypoints() const { return keypoints_; }
  const cv::Mat &getDescriptors() const { return descriptors_; }
  const cv::Mat &getImage() const { return image_; }

  // --- MapPoint associations (1:1 with keypoints_) ---
  const std::vector<MapPoint::Ptr> &getMapPoints() const { return map_points_; }
  std::vector<MapPoint::Ptr> &accessMapPoints() { return map_points_; }

  /**
   * @brief Count how many keypoints are associated with a valid (non-null,
   * non-bad) MapPoint.
   */
  size_t countMapPoints() const;

  // --- Covisibility ---
  /**
   * @brief Add a covisible KeyFrame with a weight = number of shared MapPoints.
   */
  void addCovisibleKeyFrame(KeyFrame::Ptr kf, int weight);

  /**
   * @brief Recompute covisibility edges by scanning MapPoint observations.
   *        Call this after triangulating new points or merging MapPoints.
   */
  void updateCovisibility();

  /**
   * @brief Get the N most covisible KeyFrames, sorted by weight descending.
   */
  std::vector<KeyFrame::Ptr> getBestCovisibles(int n) const;

  /**
   * @brief Get all covisible KeyFrames (unsorted).
   */
  std::vector<KeyFrame::Ptr> getAllCovisibles() const;

  /**
   * @brief Copy this KeyFrame's MapPoint associations back into a Frame.
   *        Used so that track() can propagate MapPoints to the next frame.
   */
  void syncMapPointsToFrame(std::shared_ptr<Frame> frame) const;

private:
  KeyFrame(std::shared_ptr<Frame> frame, size_t id);

  // --- Identity ---
  size_t id_;       // KeyFrame-specific sequential ID
  size_t frame_id_; // Original Frame ID (for debug / traceability)
  double timestamp_;

  // --- GTSAM ---
  gtsam::Key pose_key_;

  // --- Geometry ---
  gtsam::Pose3 pose_;

  // --- Visual data (owned copy) ---
  cv::Mat image_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;

  // --- Map structure ---
  std::vector<MapPoint::Ptr> map_points_; // 1:1 with keypoints_

  // --- Covisibility graph ---
  // Sorted: highest weight first
  std::vector<std::pair<int, KeyFrame::Ptr>> ordered_covisibles_;
  // Fast lookup
  std::map<KeyFrame::Ptr, int> covisibility_weights_;

  // --- Static ID counter ---
  static size_t next_id_;
};

} // namespace frontend