#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "Frame.hpp"

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
    FeatureMatcher(const std::string &matcher_type, float ratio_thresh = 0.75f);
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
     * @brief Project MapPoints into frame using pose guess, then match by
     * descriptor.
     *
     * @param frame          Current frame with keypoints and descriptors
     * @param map_points     Local map points to project
     * @param K              Camera intrinsics
     * @param T_w_c          Current pose guess (world → camera)
     * @param search_radius  Pixel radius for candidate search
     * @return Number of successful matches (stored in frame's map_points_
     * vector)
     */
    int matchByProjection(Frame::Ptr frame,
                          const std::vector<MapPoint::Ptr> &map_points,
                          const cv::Mat &K, const gtsam::Pose3 &T_w_c,
                          float search_radius = 15.0f);

  private:
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    float ratio_thresh_;
};

} // namespace frontend
