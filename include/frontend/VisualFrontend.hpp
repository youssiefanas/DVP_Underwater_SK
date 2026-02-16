#pragma once

#include "FeatureExtractor.hpp"
#include "FeatureMatcher.hpp"
#include "Frame.hpp"
#include "KeyFrame.hpp"
#include "Map.hpp"
#include "PoseEstimator.hpp"
#include "Viewer.hpp"
#include "VisualTypes.hpp"
#include <gtsam/geometry/Pose3.h>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

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

class VisualFrontend {
public:
    using Ptr = std::shared_ptr<VisualFrontend>;

    VisualFrontend(const ORBParams& params, const cv::Mat& K);
    ~VisualFrontend() = default;

    bool handleImage(const cv::Mat& image, double timestamp);

    bool process(Frame::Ptr current_frame);

    Frame::Ptr getLatestFrame() const;
    KeyFrame::Ptr getLatestKeyFrame() const;
    Map::Ptr getMap() const { return map_; }

  private:
    // --- Internal pipeline stages ---
    void extractFeatures(Frame::Ptr frame);

    /**
     * @brief Try to initialize the map from two frames with enough baseline.
     * @return true if initialization succeeded (enough matches, enough
     * parallax)
     */
    bool tryInitialize(Frame::Ptr current_frame);

    /**
     * @brief Main tracking loop: match against local map, estimate pose,
     *        decide whether to insert a new KeyFrame, triangulate new points.
     * @return true if tracking succeeded
     */
    bool track(Frame::Ptr current_frame);

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
    KeyFrame::Ptr last_keyframe_; // Most recent KeyFrame
    KeyFrame::Ptr
        reference_keyframe_; // Reference KF for current tracking window

    // --- Camera ---
    cv::Mat K_;

    // --- Map ---
    Map::Ptr map_;

    // --- KeyFrame insertion parameters ---
    int frames_since_last_kf_ = 0;
    static constexpr int kMinFramesBetweenKF = 5;   // Don't insert KF too fast
    static constexpr int kMinTrackedMapPoints = 50; // If below this → force KF
    static constexpr double kMinBaseline = 0.15; // meters (tune for underwater)
    static constexpr double kMinRotationDeg = 10.0; // degrees
};

} // namespace frontend