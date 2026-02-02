#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace frontend {

class PoseEstimator {
public:
    using Ptr = std::shared_ptr<PoseEstimator>;

    PoseEstimator();
    ~PoseEstimator() = default;

    /**
     * @brief Estimate relative pose (R, t) between two sets of matched points.
     * 
     * @param points_prev Points in the previous frame (2D).
     * @param points_curr Points in the current frame (2D).
     * @param K Camera intrinsics matrix.
     * @param R Output rotation matrix (3x3).
     * @param t Output translation vector (3x1).
     * @param mask Output inlier mask from RANSAC.
     * @return true if pose estimation was successful, false otherwise.
     */
    bool estimate(const std::vector<cv::Point2f>& points_prev,
                  const std::vector<cv::Point2f>& points_curr,
                  const cv::Mat& K,
                  cv::Mat& R,
                  cv::Mat& t,
                  cv::Mat& mask);
};

} // namespace frontend
