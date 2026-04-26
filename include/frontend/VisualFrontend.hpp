#pragma once

#include <chrono>
#include <memory>
#include <optional>

#include <Eigen/Core>
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

class FeatureExtractor;
class FeatureMatcher;
struct ORBParams;
class PoseEstimator;

/**
 * @brief Per-frame timing breakdown (all values in milliseconds).
 */
struct FrameTiming {
  double extraction_ms = 0.0;   ///< ORB feature extraction.
  double tracking_ms = 0.0;     ///< State machine (matching + PnP + KF insertion).
  double viewer_ms = 0.0;       ///< Viewer drawing (0 if disabled).
  double total_ms = 0.0;        ///< Total handleImage time.
};

/**
 * @brief Tracking stage of the frontend state machine.
 */
enum class Stage {
  NO_IMAGES_YET,  ///< No frames have been received yet.
  INITIALIZING,   ///< Have one frame; waiting for a second with enough baseline.
  TRACKING,       ///< Map is initialized; steady-state tracking active.
  LOST            ///< Tracking lost; attempting relocalization against known keyframes.
};

/**
 * @brief Output struct consumed by the backend (Visual Odom Node).
 *
 * Each time the frontend creates or promotes a KeyFrame it populates one of
 * these and stores it in pending_output_.
 */
struct FrontendOutput {
  Pose3d relative_pose{
      Pose3d::Identity()}; ///< Measured relative pose: T_{prev}^{-1} * T_{curr}.
  Pose3d initial_estimate{
      Pose3d::Identity()};       ///< Initial world pose of the current KeyFrame.
  double timestamp;              ///< Timestamp of the current frame (seconds).
  bool is_first_keyframe = false;///< If true, emit a PriorFactor instead of BetweenFactor.
  Pose3d prior_pose{
      Pose3d::Identity()}; ///< Prior pose (used only when is_first_keyframe is true).
  Eigen::Matrix<double, 6, 6> pose_covariance{
      Eigen::Matrix<double, 6, 6>::Identity() * 0.01}; ///< 6x6 covariance [rot(3), trans(3)].
};

/**
 * @brief Monocular visual SLAM frontend: feature tracking, map initialization,
 *        KeyFrame management, and triangulation.
 *
 * The VisualFrontend implements a three-stage state machine:
 *   1. **NO_IMAGES_YET** — waits for the first frame, creates the initial KF.
 *   2. **INITIALIZING** — waits for a second frame with enough baseline,
 *      estimates the Essential matrix, triangulates initial MapPoints.
 *   3. **TRACKING** — projection-matches against the local map, refines pose
 *      via PnP, decides whether to insert new KeyFrames, and triangulates
 *      new MapPoints.
 *
 * After each KeyFrame event, a FrontendOutput is stored internally.
 */
class VisualFrontend {
public:
    using Ptr = std::shared_ptr<VisualFrontend>;

    /**
     * @brief Construct the frontend with ORB parameters and camera intrinsics.
     * @param params        ORB feature extraction and matching parameters.
     * @param K             3x3 camera intrinsic matrix (CV_64F).
     * @param init_scale    Monocular scale factor applied during initialization
     *                      (default 1.0; must be > 0).
     * @param enable_viewer If true, create a live visualization window.
     * @param dist_coeffs   Distortion coefficients (CV_64F).
     */
    VisualFrontend(const ORBParams &params, const cv::Mat &K,
                   const cv::Mat &dist_coeffs, double init_scale = 1.0,
                   bool enable_viewer = false);
    ~VisualFrontend() = default;

    /**
     * @brief Main entry point: feed a new grayscale image into the pipeline.
     *
     * Extracts features, dispatches to the current pipeline stage, and
     * optionally updates the viewer.
     *
     * @param image           Grayscale input image (must not be empty).
     * @param timestamp       Capture time of the image (seconds).
     * @param attitude_prior  Optional camera-frame rotation derived from IMU
     *                        (R_world_camera). When present, motion-only BA
     *                        will add a roll/pitch prior factor.
     * @return true if the frame was successfully processed (tracking valid).
     */
    bool handleImage(const cv::Mat& image, double timestamp,
                     std::optional<Eigen::Matrix3d> attitude_prior =
                         std::nullopt);

    /**
     * @brief Configure the IMU roll/pitch prior used in motion-only BA.
     * @param roll_sigma_rad   Std-dev for roll (radians). 0 disables.
     * @param pitch_sigma_rad  Std-dev for pitch (radians). 0 disables.
     */
    void setImuPriorSigmas(double roll_sigma_rad, double pitch_sigma_rad);

    /**
     * @brief Process a pre-built Frame through the state machine.
     *
     * This is the internal entry point called by handleImage() after feature
     * extraction. Can also be called directly for testing.
     *
     * @param current_frame  The frame to process (must have features set).
     * @return true if tracking succeeded for this frame.
     */
    bool process(Frame::Ptr current_frame);

    /// @brief Get the most recently processed Frame.
    Frame::Ptr getLatestFrame() const;

    /// @brief Get the most recently created KeyFrame.
    KeyFrame::Ptr getLatestKeyFrame() const;

    /// @brief Get the global Map shared pointer.
    Map::Ptr getMap() const { return map_; }

    /// @brief Consume the pending FrontendOutput (returns nullopt if none).
    std::optional<FrontendOutput> consumeBackendOutput();

    /// @brief Get the latest pose covariance (updated every tracked frame).
    const Eigen::Matrix<double, 6, 6> &getLastCovariance() const {
        return last_covariance_;
    }

    /// @brief Get per-frame timing breakdown from the last handleImage call.
    const FrameTiming &getLastTiming() const { return last_timing_; }

private:
    // ─── Internal pipeline stages ────────────────────────────────

    /**
     * @brief Extract ORB features from the given frame.
     * @param frame  Frame to extract features from (modified in-place).
     */
    void extractFeatures(Frame::Ptr frame);

    /**
     * @brief Attempt two-view initialization from the reference KeyFrame.
     *
     * Matches features between last_keyframe_ and @p current_frame, estimates
     * the Essential matrix, checks baseline and parallax, triangulates initial
     * MapPoints, and (if enough points pass validation) finalizes the map.
     *
     * @param current_frame  The candidate second frame for initialization.
     * @return true if initialization succeeded and the stage transitions to TRACKING.
     */
    bool tryInitialize(Frame::Ptr current_frame);

    /**
     * @brief Main tracking entry point: predict pose, match, solve PnP,
     *        and optionally insert a new KeyFrame.
     * @param current_frame  The frame to track.
     * @return true if tracking succeeded.
     */
    bool track(Frame::Ptr current_frame);

    /**
     * @brief Core tracking routine: multi-phase matching against the local map
     *        followed by PnP pose estimation.
     *
     * Phases:
     *   1. Propagate MapPoints from the previous frame via descriptor matching.
     *   2. Project local map MapPoints into the frame (narrow radius).
     *   3. Widen the search radius if too few matches.
     *   4. Relocalize against the reference KeyFrame if still insufficient.
     *   5. Solve PnP with RANSAC, reject outliers.
     *   6. (Optional) re-project with the refined pose and re-solve PnP.
     *   7. Reject implausible pose jumps.
     *
     * @param current_frame  The frame being tracked (pose updated in-place).
     * @return true if enough inliers and the pose passes sanity checks.
     */
    bool trackWithLocalMap(Frame::Ptr current_frame);

    /**
     * @brief Predict the current frame's pose using the constant-velocity model.
     * @param current_frame  Frame whose pose will be set (modified in-place).
     */
    void predictPose(Frame::Ptr current_frame);

    /**
     * @brief Decide whether the current frame should be promoted to a KeyFrame.
     *
     * Criteria: minimum inter-KF frame count, sufficient baseline or rotation
     * relative to the last KeyFrame, or low tracked MapPoint count.
     *
     * @param current_frame  The candidate frame.
     * @return true if a new KeyFrame should be inserted.
     */
    bool shouldInsertKeyFrame(Frame::Ptr current_frame) const;

    /**
     * @brief Promote the current frame to KeyFrame, triangulate new MapPoints,
     *        update covisibility, and emit a FrontendOutput for the backend.
     * @param current_frame  The frame to promote.
     */
    void insertKeyFrame(Frame::Ptr current_frame);

    /**
     * @brief Triangulate new MapPoints between two KeyFrames.
     *
     * Matches descriptors internally, then delegates to the overload that
     * accepts pre-computed matches.
     *
     * @param kf1  First KeyFrame (query side of matching).
     * @param kf2  Second KeyFrame (train side of matching).
     */
    void triangulateNewPoints(KeyFrame::Ptr kf1, KeyFrame::Ptr kf2);

    /**
     * @brief Triangulate new MapPoints from pre-matched correspondences.
     *
     * For each match:
     *   - If both KeyFrames already have a MapPoint, skip.
     *   - If only one has a MapPoint, propagate it to the other.
     *   - If neither has one, triangulate and validate the new 3D point.
     *
     * @param kf1      First KeyFrame.
     * @param kf2      Second KeyFrame.
     * @param matches  Pre-computed descriptor matches between kf1 and kf2.
     */
    void triangulateNewPoints(KeyFrame::Ptr kf1, KeyFrame::Ptr kf2,
                              const std::vector<cv::DMatch> &matches);

    /**
     * @brief Count keypoints in this frame that have a valid (non-null, non-bad)
     *        MapPoint association.
     * @param frame  Frame to inspect.
     * @return Number of actively tracked MapPoints.
     */
    size_t countTrackedMapPoints(Frame::Ptr frame) const;

    /**
     * @brief Filter matches by index validity and bounds, extracting matched
     *        2D point pairs.
     *
     * Discards matches with negative indices or indices out of bounds for
     * the keypoint vectors. Outputs are populated in lockstep.
     *
     * @param matches  Raw descriptor matches.
     * @param kps1     Keypoints of the query image.
     * @param kps2     Keypoints of the train image.
     * @param[out] pts1  Filtered 2D points from kps1.
     * @param[out] pts2  Filtered 2D points from kps2.
     * @return Subset of @p matches that passed validation.
     */
    static std::vector<cv::DMatch> extractValidMatchedPoints(
        const std::vector<cv::DMatch> &matches,
        const std::vector<cv::KeyPoint> &kps1,
        const std::vector<cv::KeyPoint> &kps2,
        std::vector<cv::Point2f> &pts1,
        std::vector<cv::Point2f> &pts2);

    /**
     * @brief Validate a triangulated 3D point against quality thresholds.
     *
     * Checks: finite coordinates, positive depth in both camera frames,
     * distance within @p max_distance, and parallax angle >= @p min_parallax_deg.
     *
     * @param p               Triangulated point in camera 1's local frame.
     * @param R_cv            3x3 rotation from camera 1 to camera 2 (CV_64F).
     * @param t_cv            3x1 translation from camera 1 to camera 2 (CV_64F).
     * @param cam1_world      World-frame position of camera 1.
     * @param cam2_world      World-frame position of camera 2.
     * @param T_w_ref         Transform from camera 1's local frame to world.
     * @param max_distance    Maximum allowed distance from the camera (map units).
     * @param min_parallax_deg  Minimum required parallax angle (degrees).
     * @return true if the point passes all checks.
     */
    static bool isTriangulatedPointValid(
        const cv::Point3f &p, const cv::Mat &R_cv, const cv::Mat &t_cv,
        const Eigen::Vector3d &cam1_world, const Eigen::Vector3d &cam2_world,
        const Pose3d &T_w_ref, double max_distance, double min_parallax_deg);

    /**
     * @brief Propagate MapPoint associations from last_frame_ to the current frame.
     *
     * Matches descriptors between last_frame_ and @p current_frame, then copies
     * valid (non-null, non-bad) MapPoints to the corresponding indices.
     *
     * @param current_frame  Frame receiving propagated MapPoints.
     * @return Number of MapPoints successfully propagated.
     */
    size_t propagateMapPoints(Frame::Ptr current_frame);

    /**
     * @brief Descriptor-based relocalization against the reference KeyFrame.
     *
     * Used as a fallback when projection matching yields too few correspondences.
     * Matches reference_keyframe_ descriptors against @p current_frame and copies
     * MapPoints for unoccupied keypoint slots.
     *
     * @param current_frame  Frame to relocalize.
     */
    void relocateFromReferenceKF(Frame::Ptr current_frame);

    /**
     * @brief Run PnP RANSAC for initial pose + outlier rejection.
     *
     * Delegates to PoseEstimator::estimateRefined(). Logs and returns false
     * if PnP fails or the inlier count is below kMinPnPInliers.
     * Does NOT compute covariance — that is handled by motionOnlyBA.
     *
     * @param current_frame       Frame whose pose will be refined.
     * @param[out] pnp_inliers    Number of PnP inliers on success.
     * @return true if PnP succeeded with enough inliers.
     */
    bool solvePnP(Frame::Ptr current_frame, int &pnp_inliers);

    /**
     * @brief Reject implausible pose jumps between the predicted and actual pose.
     *
     * Computes the translation and rotation delta between @p predicted_pose and
     * @p actual_pose, compares against adaptive thresholds (which may be relaxed
     * for strong PnP solutions or high-velocity motion).
     *
     * @param predicted_pose  The pose predicted by the velocity model.
     * @param actual_pose     The pose estimated by PnP.
     * @param pnp_inliers    Number of PnP inliers (used for threshold adaptation).
     * @return true if the pose delta is within acceptable bounds.
     */
    bool validatePoseJump(const Pose3d &predicted_pose,
                          const Pose3d &actual_pose, int pnp_inliers) const;

    /**
     * @brief Populate and store a FrontendOutput for the backend to consume.
     * @param is_first   True if this is the very first KeyFrame (emit PriorFactor).
     * @param relative   Relative pose between the previous and current KeyFrame.
     * @param estimate   Initial world-frame pose of the current KeyFrame.
     * @param timestamp  Capture timestamp of the current frame (seconds).
     */
    void emitBackendOutput(bool is_first, const Pose3d &relative,
                           const Pose3d &estimate, double timestamp,
                           const Eigen::Matrix<double, 6, 6> &covariance =
                               Eigen::Matrix<double, 6, 6>::Identity() * 0.01);

    /**
     * @brief Undo a failed initialization attempt.
     *
     * Marks all MapPoints in @p curr_kf as bad, removes them from the global
     * map, clears bad references in last_keyframe_, and removes the KeyFrame.
     *
     * @param curr_kf  The KeyFrame created during the failed initialization.
     */
    void rollbackFailedInitialization(KeyFrame::Ptr curr_kf);

    /**
     * @brief Attempt relocalization by matching against recent keyframes.
     *
     * Iterates over all keyframes in the map, matches descriptors against
     * the current frame, and attempts PnP. Returns true if a valid pose
     * is recovered.
     *
     * @param current_frame  The frame to relocalize.
     * @return true if relocalization succeeded.
     */
    bool tryRelocalize(Frame::Ptr current_frame);

    /**
     * @brief Reset the frontend to re-initialize from the current frame.
     *
     * Called when relocalization fails for too long. Clears the map and
     * transitions back to NO_IMAGES_YET, using the last known good pose
     * as the starting point for the new map segment.
     */
    void resetToInitializing();

private:
    // ─── State ───────────────────────────────────────────────────
    Stage stage_;  ///< Current pipeline stage.

    // ─── Components ──────────────────────────────────────────────
    std::shared_ptr<FeatureExtractor> feature_extractor_;  ///< ORB feature detector.
    std::shared_ptr<Viewer> viewer_;                       ///< Optional live viewer (null if disabled).
    std::shared_ptr<FeatureMatcher> feature_matcher_;      ///< Descriptor + projection matcher.
    std::shared_ptr<PoseEstimator> pose_estimator_;        ///< Essential / PnP / triangulation.

    // ─── Frame history ───────────────────────────────────────────
    Frame::Ptr last_frame_;              ///< Most recently processed Frame.
    Frame::Ptr current_frame_;           ///< Frame currently being processed.
    KeyFrame::Ptr last_keyframe_;        ///< Most recently created KeyFrame.
    KeyFrame::Ptr reference_keyframe_;   ///< Reference KF for the current tracking window.

    // ─── Camera ──────────────────────────────────────────────────
    cv::Mat K_;              ///< 3x3 camera intrinsic matrix.
    double init_scale_ = 1.0; ///< Monocular scale factor for initialization.

    // ─── Map ─────────────────────────────────────────────────────
    Map::Ptr map_;  ///< Global map of KeyFrames and MapPoints.

    // ─── Velocity model ──────────────────────────────────────────
    Pose3d velocity_{Pose3d::Identity()}; ///< Last inter-frame relative transform (T_{k-1,k}).
    bool has_velocity_ = false;           ///< True once at least one velocity estimate exists.
    int consecutive_failures_ = 0;        ///< Consecutive frames where tracking failed.
    int lost_frames_ = 0;                 ///< Frames spent in LOST state.
    Pose3d last_good_pose_{Pose3d::Identity()};  ///< Last successfully tracked pose (for recovery).

    /// Pending output for the backend (consumed by the VO node).
    std::optional<FrontendOutput> pending_output_;

    /// Latest pose covariance from PnP (updated each successful track).
    Eigen::Matrix<double, 6, 6> last_covariance_{
        Eigen::Matrix<double, 6, 6>::Identity() * 0.03};

    /// Per-frame timing breakdown (populated by handleImage).
    FrameTiming last_timing_;

    // ─── KeyFrame insertion thresholds ───────────────────────────
    int frames_since_last_kf_ = 0;                          ///< Frame counter since last KF.
    static constexpr int kMinFramesBetweenKF = 5;           ///< Min frames before allowing a new KF.
    static constexpr double kMinBaseline = 0.15;            ///< Min translation to trigger KF (meters).
    static constexpr double kMinRotationDeg = 10.0;         ///< Min rotation to trigger KF (degrees).

    // ─── Tracking / PnP thresholds ──────────────────────────────
    static constexpr double kMaxPoseJumpTranslation = 0.15; ///< Max allowed pose-jump translation (meters).
    static constexpr double kMaxPoseJumpRotationDeg =
        10.0; ///< Max allowed jump rotation(degrees).
    static constexpr int kMaxConsecutiveFailures = 5;       ///< Failures before transitioning to LOST.
    static constexpr int kMaxRelocFrames = 30;              ///< Max frames in LOST before re-initialization.
    static constexpr int kMinInitMapPoints = 25;            ///< Min triangulated points for init to succeed.
    static constexpr int kMinTrackedMapPoints = 30;         ///< Below this count, force a new KF.
    static constexpr int kMinProjectionMatches = 12;        ///< Min projection matches before widening search.
    static constexpr int kMinPnPInliers = 15;               ///< Min PnP inliers to accept a pose.
    static constexpr int kStrongPnPInliers = 50;            ///< Inlier count that relaxes jump thresholds.
    static constexpr float kSearchRadius = 40.0f;           ///< Narrow projection search radius (pixels).
    static constexpr float kSearchRadiusWide = 80.0f;       ///< Wide fallback search radius (pixels).
    static constexpr int kMinTrackedForNewKF = 20;          ///< Min tracked MPs to allow a new KF.

    // ─── Triangulation quality thresholds ────────────────────────
    static constexpr float kMatchRatioThreshold = 0.7f;     ///< Lowe's ratio test threshold.
    static constexpr double kMaxTriangulationDist = 50.0;   ///< Max point distance from camera (map units).
    static constexpr double kMinParallaxDeg = 1.0;          ///< Min parallax angle for triangulation (degrees).
    static constexpr size_t kMinMatchesForInit = 20;        ///< Min valid matches to attempt initialization.
};

} // namespace frontend
