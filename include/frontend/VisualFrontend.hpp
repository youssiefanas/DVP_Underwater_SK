#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "Frame.hpp"
#include "FeatureExtractor.hpp"
#include "FeatureMatcher.hpp"
#include "VisualTypes.hpp"
#include "Viewer.hpp"
#include "PoseEstimator.hpp"
#include <gtsam/geometry/Pose3.h>

namespace frontend {

// Forward declarations
class FeatureExtractor;
class FeatureMatcher;
struct ORBParams;
class PoseEstimator;

// Tracking Stage
enum class Stage {
    NO_IMAGES_YET,
    INITIALIZED,
    TRACKING
};

class VisualFrontend {
public:
    using Ptr = std::shared_ptr<VisualFrontend>;

    VisualFrontend(const ORBParams& params, const cv::Mat& K);
    ~VisualFrontend() = default;

    bool handleImage(const cv::Mat& image, double timestamp);

    bool process(Frame::Ptr current_frame);
private:
    /**
     * @brief Internal processing logic (State Machine)
     */
    void extractFeatures(Frame::Ptr frame);

private:
    // --- State Management ---
    Stage stage_;

    // --- Components ---
    std::shared_ptr<FeatureExtractor> feature_extractor_;
    std::shared_ptr<Viewer> viewer_;

    // --- Data Management ---
    Frame::Ptr last_frame_;
    Frame::Ptr current_frame_; // Optional: Keep track if needed elsewhere
    
    // Feature Matcher
    std::shared_ptr<FeatureMatcher> feature_matcher_;

    // Pose Estimator
    std::shared_ptr<PoseEstimator> pose_estimator_;
    
    // Camera Intrinsics
    cv::Mat K_;
};


} // namespace frontend