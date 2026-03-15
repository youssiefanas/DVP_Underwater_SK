#pragma once

#include <memory>
#include <optional>

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
  Pose3d relative_pose{
      Pose3d::Identity()}; ///< Measured relative pose: T_{prev}^{-1} * T_{curr}
  Pose3d initial_estimate{
      Pose3d::Identity()};       ///< Initial world pose of current KeyFrame
  double timestamp;              ///< Timestamp of the current frame
  bool is_first_keyframe = false;///< If true, add a PriorFactor instead of BetweenFactor
  Pose3d prior_pose{
      Pose3d::Identity()}; ///< Used only if is_first_keyframe is true
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
     * @brief Triangulate new MapPoints between two KeyFrames.
     *        Matches descriptors internally, then triangulates.
     */
    void triangulateNewPoints(KeyFrame::Ptr kf1, KeyFrame::Ptr kf2);

    /**
     * @brief Triangulate new MapPoints using pre-matched correspondences.
     *        Used by initialization with geometrically verified inliers.
     */
    void triangulateNewPoints(KeyFrame::Ptr kf1, KeyFrame::Ptr kf2,
                              const std::vector<cv::DMatch> &matches);

    /**
     * @brief Count how many keypoints in this frame are already associated
     *        with a valid MapPoint.
     */
    size_t countTrackedMapPoints(Frame::Ptr frame) const;

    /**
     * @brief Filter matches by index validity and bounds, and extract
     *        corresponding 2D point pairs.
     * @return Filtered matches; pts1/pts2 are populated in lockstep.
     */
    static std::vector<cv::DMatch> extractValidMatchedPoints(
        const std::vector<cv::DMatch> &matches,
        const std::vector<cv::KeyPoint> &kps1,
        const std::vector<cv::KeyPoint> &kps2,
        std::vector<cv::Point2f> &pts1,
        std::vector<cv::Point2f> &pts2);

    /**
     * @brief Validate a triangulated 3D point: finite, positive depth in both
     *        cameras, within max distance, and sufficient parallax angle.
     */
    static bool isTriangulatedPointValid(
        const cv::Point3f &p, const cv::Mat &R_cv, const cv::Mat &t_cv,
        const Eigen::Vector3d &cam1_world, const Eigen::Vector3d &cam2_world,
        const Pose3d &T_w_ref, double max_distance, double min_parallax_deg);

    /// Propagate MapPoints from last_frame_ to current_frame via matching.
    size_t propagateMapPoints(Frame::Ptr current_frame);

    /// Relocate against reference keyframe when projection matching is weak.
    void relocateFromReferenceKF(Frame::Ptr current_frame);

    /// Run PnP with inlier threshold check. Returns false on failure.
    bool solvePnP(Frame::Ptr current_frame, int &pnp_inliers);

    /// Reject implausible pose jumps relative to the predicted pose.
    bool validatePoseJump(const Pose3d &predicted_pose,
                          const Pose3d &actual_pose, int pnp_inliers) const;

    /// Emit a FrontendOutput for the backend to consume.
    void emitBackendOutput(bool is_first, const Pose3d &relative,
                           const Pose3d &estimate, double timestamp);

    /// Undo a failed initialization: mark MapPoints bad and remove KF.
    void rollbackFailedInitialization(KeyFrame::Ptr curr_kf);

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
    Pose3d velocity_{Pose3d::Identity()}; // T_{k-1, k} relative transform
    bool has_velocity_ = false;
    int consecutive_failures_ = 0;

    /// Pending output for the backend (consumed by vo_node)
    std::optional<FrontendOutput> pending_output_;

    // --- KeyFrame insertion parameters ---
    int frames_since_last_kf_ = 0;
    static constexpr int kMinFramesBetweenKF = 5;
    static constexpr double kMinBaseline = 0.15;      // meters
    static constexpr double kMinRotationDeg = 10.0;   // degrees

    // --- Tracking / PnP parameters ---
    static constexpr double kMaxPoseJumpTranslation = 0.20; // meters
    static constexpr double kMaxPoseJumpRotationDeg = 12.0;  // degrees
    static constexpr int kMaxConsecutiveFailures = 10;
    static constexpr int kMinInitMapPoints = 25;
    static constexpr int kMinTrackedMapPoints = 30;
    static constexpr int kMinProjectionMatches = 12;
    static constexpr int kMinPnPInliers = 15;
    static constexpr int kStrongPnPInliers = 50;
    static constexpr float kSearchRadius = 40.0f;     // pixels
    static constexpr float kSearchRadiusWide = 80.0f;  // pixels, fallback
    static constexpr int kMinTrackedForNewKF = 20;

    // --- Triangulation quality thresholds ---
    static constexpr float kMatchRatioThreshold = 0.7f;
    static constexpr double kMaxTriangulationDist = 50.0;  // map units
    static constexpr double kMinParallaxDeg = 1.0;         // degrees
    static constexpr size_t kMinMatchesForInit = 20;
};

} // namespace frontend
