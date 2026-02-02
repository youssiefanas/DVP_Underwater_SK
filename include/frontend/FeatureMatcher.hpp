#pragma once

#include <memory>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d.hpp>
#include "Frame.hpp"
#include "MapPoint.hpp" // Added dependency

namespace frontend {

class FeatureMatcher {
public:
    using Ptr = std::shared_ptr<FeatureMatcher>;

    /**
     * @brief Construct a new Feature Matcher object
     * 
     * @param matcher_type "NORM_HAMMING" or "NORM_L2"
     * @param ratio_thresh Ratio test threshold (default 0.7)
     */
    FeatureMatcher(const std::string& matcher_type, float ratio_thresh = 0.7f);
    ~FeatureMatcher() = default;

    /**
     * @brief Match descriptors between two frames.
     * 
     * @param frame1 Previous frame (query)
     * @param frame2 Current frame (train)
     * @return std::vector<cv::DMatch> List of good matches
     */
    std::vector<cv::DMatch> match(Frame::Ptr frame1, Frame::Ptr frame2);

    /**
     * @brief Match features in current frame to the Local Map using projection.
     * 
     * @param current Frame to match (must have pose guess)
     * @param local_map_points Vector of MapPoints to search for
     * @param radius Search radius in pixels
     * @return Number of matches found
     */
    int matchProjection(Frame::Ptr current, const std::vector<MapPoint::Ptr>& local_map_points, float radius = 5.0f);

private:
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    float ratio_thresh_;
};

} // namespace frontend
