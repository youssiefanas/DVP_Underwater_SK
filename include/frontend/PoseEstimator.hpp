#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "Frame.hpp"

namespace frontend {

class PoseEstimator {
public:
    using Ptr = std::shared_ptr<PoseEstimator>;

    PoseEstimator();
    ~PoseEstimator() = default;

    /// Store camera intrinsics (call once during construction).
    void setIntrinsics(const cv::Mat &K);

    /**
     * Estimate relative pose via Essential matrix (2D-2D).
     * Only used during initialization (no map yet).
     */
    bool estimate(const std::vector<cv::Point2f>& points_prev,
                  const std::vector<cv::Point2f>& points_curr,
                  const cv::Mat& K,
                  cv::Mat& R,
                  cv::Mat& t,
                  cv::Mat& mask);

    /**
     * Refine pose using PnP with MapPoint observations (3D-2D).
     * Reads frame's MapPoint associations, runs solvePnPRansac
     * with the frame's current pose as initial guess.
     * On success, updates the frame's pose and removes outliers.
     */
    bool estimateRefined(Frame::Ptr frame, int *inlier_count = nullptr);

    /**
     * Triangulate 3D points from two sets of 2D correspondences.
     */
    bool triangulate(const std::vector<cv::Point2f> &pts_prev,
                     const std::vector<cv::Point2f> &pts_curr, const cv::Mat &K,
                     const cv::Mat &R, const cv::Mat &t,
                     std::vector<cv::Point3f> &points_3d);

  private:
    cv::Mat K_;
    cv::Mat dist_coeffs_;
};

} // namespace frontend