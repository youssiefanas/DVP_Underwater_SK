#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "MapPoint.hpp"
#include "Pose3d.hpp"

namespace frontend {

class Frame;

/**
 * @brief A selected Frame promoted to anchor the map and covisibility graph.
 *
 * KeyFrames are a sparse subset of Frames chosen for their spatial diversity
 * (sufficient baseline or rotation from the previous KeyFrame). They own
 * deep copies of the image, keypoints, and descriptors from the originating
 * Frame, and maintain:
 *   - A 1:1 MapPoint association vector (same indexing as keypoints).
 *   - A covisibility graph linking KeyFrames that share MapPoints.
 *
 * KeyFrames are the nodes in the factor graph consumed by the GTSAM backend.
 * They are created exclusively through the create() factory method.
 */
class KeyFrame : public std::enable_shared_from_this<KeyFrame> {
public:
  using Ptr = std::shared_ptr<KeyFrame>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Factory: promote a Frame into a KeyFrame.
   *
   * Copies features, descriptors, pose, image, and MapPoint associations
   * from the source frame. Assigns a monotonically increasing KeyFrame ID.
   *
   * @param frame  The source Frame to promote (must not be null).
   * @return Shared pointer to the new KeyFrame, or nullptr if @p frame is null.
   */
  static Ptr create(std::shared_ptr<Frame> frame);

  /// @brief Get the unique KeyFrame-specific sequential ID.
  size_t getId() const { return id_; }

  /// @brief Get the original Frame ID (for debug / traceability).
  size_t getFrameId() const { return frame_id_; }

  /// @brief Get the capture timestamp in seconds.
  double getTimestamp() const { return timestamp_; }

  /// @brief Get the current world-frame pose (T_world_camera), read-only.
  const Pose3d &getPose() const { return pose_; }

  /**
   * @brief Update the KeyFrame's pose (e.g. after backend optimization).
   * @param pose  New SE(3) transform T_world_camera.
   */
  void setPose(const Pose3d &pose) { pose_ = pose; }

  /// @brief Get the detected keypoints (read-only, copied from source Frame).
  const std::vector<cv::KeyPoint> &getKeypoints() const { return keypoints_; }

  /// @brief Get the descriptor matrix (read-only, shallow copy from source Frame).
  const cv::Mat &getDescriptors() const { return descriptors_; }

  /// @brief Get the stored image (read-only, shallow copy from source Frame).
  const cv::Mat &getImage() const { return image_; }

  /// @brief Get the MapPoint association vector (read-only, 1:1 with keypoints).
  const std::vector<MapPoint::Ptr> &getMapPoints() const { return map_points_; }

  /// @brief Get mutable access to the MapPoint association vector.
  std::vector<MapPoint::Ptr> &accessMapPoints() { return map_points_; }

  /**
   * @brief Count how many keypoints are associated with a valid (non-null,
   *        non-bad) MapPoint.
   * @return Number of active MapPoint associations.
   */
  size_t countMapPoints() const;

  /**
   * @brief Register this KeyFrame as an observer on all its valid MapPoints.
   *
   * Must be called after create() so that shared_from_this() is valid.
   * Iterates over map_points_ and calls MapPoint::addObservation for each
   * non-null, non-bad entry.
   */
  void registerMapPointObservations();

  /**
   * @brief Manually add a covisibility edge to another KeyFrame.
   * @param kf      The covisible KeyFrame (ignored if null or self).
   * @param weight  Number of shared MapPoints (edge weight).
   */
  void addCovisibleKeyFrame(KeyFrame::Ptr kf, int weight);

  /**
   * @brief Recompute all covisibility edges by scanning MapPoint observations.
   *
   * Clears existing edges, then for each valid MapPoint counts how many
   * keypoints are shared with each other KeyFrame. Only edges with weight
   * >= kMinCovisibilityWeight are retained. Results are sorted by weight
   * descending.
   *
   * Call this after triangulating new points or merging MapPoints.
   */
  void updateCovisibility();

  /**
   * @brief Get the N most covisible KeyFrames, sorted by weight descending.
   * @param n  Maximum number of KeyFrames to return.
   * @return Vector of up to @p n covisible KeyFrames.
   */
  std::vector<KeyFrame::Ptr> getBestCovisibles(int n) const;

  /**
   * @brief Get all covisible KeyFrames, sorted by weight descending.
   * @return Vector of all KeyFrames with weight >= kMinCovisibilityWeight.
   */
  std::vector<KeyFrame::Ptr> getAllCovisibles() const;

  /**
   * @brief Copy this KeyFrame's MapPoint associations back into a Frame.
   *
   * Overwrites the Frame's MapPoint vector with this KeyFrame's version,
   * which is authoritative after triangulation. Only copies non-null,
   * non-bad entries.
   *
   * @param frame  Target Frame to update (no-op if null).
   */
  void syncMapPointsToFrame(std::shared_ptr<Frame> frame) const;

private:
  KeyFrame(std::shared_ptr<Frame> frame, size_t id);

  size_t id_;       ///< KeyFrame-specific sequential ID.
  size_t frame_id_; ///< Original Frame ID (for debug / traceability).
  double timestamp_; ///< Capture timestamp in seconds.

  Pose3d pose_; ///< World-frame camera pose (T_world_camera).

  cv::Mat image_;                          ///< Grayscale image (shallow copy, cv::Mat ref-counted).
  std::vector<cv::KeyPoint> keypoints_;    ///< Detected keypoints.
  cv::Mat descriptors_;                    ///< Descriptor matrix (shallow copy).

  std::vector<MapPoint::Ptr> map_points_; ///< 1:1 with keypoints_.

  /// Covisible KeyFrames sorted by weight (highest first).
  std::vector<std::pair<int, KeyFrame::Ptr>> ordered_covisibles_;
  /// Fast weight lookup by KeyFrame pointer.
  std::map<KeyFrame::Ptr, int> covisibility_weights_;

  static size_t next_id_;

  /// Minimum shared MapPoints required to form a covisibility edge.
  static constexpr int kMinCovisibilityWeight = 5;
};

} // namespace frontend
