#pragma once

#include <memory>
#include <optional>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "FeatureExtractor.hpp"
#include "FeatureMatcher.hpp"
#include "Frame.hpp"
#include "KeyFrame.hpp"
#include "Map.hpp"
#include "PoseEstimator.hpp"
#include "Viewer.hpp"
#include "VisualTypes.hpp"

namespace frontend {

// Forward declarations
class FeatureExtractor;
class FeatureMatcher;
struct ORBParams;
class PoseEstimator;

// Tracking Stage
enum class Stage {
  NO_IMAGES_YET,
  INITIALIZING, // Have one frame, waiting for second with enough baseline
  TRACKING
};

/**
 * @brief Output struct to be consumed by the backend (Visual Odom Node)
 */
struct FrontendOutput {
  gtsam::Key previous_key;       ///< Key of the previous KeyFrame
  gtsam::Key current_key;        ///< Key of the current (new) KeyFrame
  gtsam::Pose3 relative_pose;    ///< Measured relative pose: T_{prev}^{-1} * T_{curr}
  gtsam::Pose3 initial_estimate; ///< Initial world pose of current KeyFrame
  double timestamp;              ///< Timestamp of the current frame
  bool is_first_keyframe = false;///< If true, add a PriorFactor instead of BetweenFactor
  gtsam::Pose3 prior_pose;       ///< Used only if is_first_keyframe is true
};

class VisualFrontend {
public:
    using Ptr = std::shared_ptr<VisualFrontend>;

    VisualFrontend(const ORBParams &params, const cv::Mat &K,
                   double init_scale = 1.0, bool enable_viewer = false);
    ~VisualFrontend() = default;

    /**
     * @brief Main entry point for a new image.
     * @return true if the image was processed successfully (tracking valid).
     */
    bool handleImage(const cv::Mat& image, double timestamp);

    /**
     * @brief Process a specific Frame structure (internal entry point).
     */
    bool process(Frame::Ptr current_frame);

    // --- Getters ---
    Frame::Ptr getLatestFrame() const;
    KeyFrame::Ptr getLatestKeyFrame() const;
    Map::Ptr getMap() const { return map_; }

    /**
     * @brief Retrieve pending backend output (if any) and clear it.
     */
    std::optional<FrontendOutput> consumeBackendOutput();

private:
    // --- Internal pipeline stages ---
    void extractFeatures(Frame::Ptr frame);

    /**
     * @brief Try to initialize the map from two frames with enough baseline.
     * @return true if initialization succeeded (enough matches, enough parallax).
     */
    bool tryInitialize(Frame::Ptr current_frame);

    /**
     * @brief Main tracking loop: match against local map, estimate pose,
     *        decide whether to insert a new KeyFrame, triangulate new points.
     * @return true if tracking succeeded.
     */
    bool track(Frame::Ptr current_frame);

    /**
     * @brief Try tracking with projection matching + PnP.
     * @return true if enough MapPoints were matched and PnP succeeded.
     */
    bool trackWithLocalMap(Frame::Ptr current_frame);

    /**
     * @brief Predict the current frame's pose using constant velocity model.
     */
    void predictPose(Frame::Ptr current_frame);

    /**
     * @brief Decide whether the current frame should be promoted to KeyFrame.
     */
    bool shouldInsertKeyFrame(Frame::Ptr current_frame) const;

    /**
     * @brief Promote frame to KeyFrame, triangulate new MapPoints between it
     *        and the previous KeyFrame, update covisibility.
     */
    void insertKeyFrame(Frame::Ptr current_frame);

    /**
     * @brief Triangulate new MapPoints between two KeyFrames using their
     *        matched but not-yet-triangulated features.
     */
    void triangulateNewPoints(KeyFrame::Ptr kf1, KeyFrame::Ptr kf2);

    /**
     * @brief Count how many keypoints in this frame are already associated
     *        with a valid MapPoint.
     */
    size_t countTrackedMapPoints(Frame::Ptr frame) const;

private:
    // --- State Management ---
    Stage stage_;

    // --- Components ---
    std::shared_ptr<FeatureExtractor> feature_extractor_;
    std::shared_ptr<Viewer> viewer_;
    std::shared_ptr<FeatureMatcher> feature_matcher_;
    std::shared_ptr<PoseEstimator> pose_estimator_;

    // --- Data Management ---
    Frame::Ptr last_frame_;
    Frame::Ptr current_frame_;
    KeyFrame::Ptr last_keyframe_;      // Most recent KeyFrame
    KeyFrame::Ptr reference_keyframe_; // Reference KF for current tracking window

    // --- Camera ---
    cv::Mat K_;
    double init_scale_ = 1.0;

    // --- Map ---
    Map::Ptr map_;

    // --- Velocity model (for pose prediction) ---
    gtsam::Pose3 velocity_; // T_{k-1, k} relative transform
    bool has_velocity_ = false;
    int consecutive_failures_ = 0;

    /// Pending output for the backend (consumed by vo_node)
    std::optional<FrontendOutput> pending_output_;

    // --- KeyFrame insertion parameters ---
    // Potential Future Work: Move these to a config file.
    int frames_since_last_kf_ = 0;
    static constexpr int kMinFramesBetweenKF = 5;    // Don't insert KF too fast
    // static constexpr int kMinInitMapPoints = 40;     // Reject weak
    // initialization static constexpr int kMinTrackedMapPoints = 50;  // If
    // below this → force KF
    static constexpr double kMinBaseline = 0.15;     // meters
    static constexpr double kMinRotationDeg = 10.0;  // degrees

    // --- Projection matching parameters ---
    // static constexpr int kMinProjectionMatches = 20; // Min matches to trust
    // PnP static constexpr int kMinPnPInliers = 25;        // Reject weak PnP
    // solutions static constexpr int kStrongPnPInliers = 80;     // Allow
    // larger motion if very strong static constexpr float kSearchRadius
    // = 25.0f;    // pixels static constexpr float kSearchRadiusWide = 50.0f;//
    // fallback wider search static constexpr int kMinTrackedForNewKF = 35;   //
    // Avoid weak keyframe insertion
    static constexpr double kMaxPoseJumpTranslation = 0.20; // meters (map scale)
    static constexpr double kMaxPoseJumpRotationDeg = 12.0; // degrees
    static constexpr int kMaxConsecutiveFailures = 10;

    static constexpr int kMinInitMapPoints = 25;      // was 40
    static constexpr int kMinTrackedMapPoints = 30;   // was 50
    static constexpr int kMinProjectionMatches = 12;  // was 20
    static constexpr int kMinPnPInliers = 15;         // was 25
    static constexpr int kStrongPnPInliers = 50;      // was 80
    static constexpr float kSearchRadius = 40.0f;     // was 25.0f
    static constexpr float kSearchRadiusWide = 80.0f; // was 50.0f
    static constexpr int kMinTrackedForNewKF = 20;    // was 35
};

} // namespace frontend
