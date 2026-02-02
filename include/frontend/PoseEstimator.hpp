#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "Frame.hpp" // Added dependency

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

    /**
     * @brief Refine pose using PnP with MapPoint observations.
     * Phase 2: Local Refinement.
     * 
     * @param frame Frame with observations and initial pose guess.
     * @return true if refinement successful (enough inliers).
     */
    bool estimateRefined(Frame::Ptr frame);

    /**
     * @brief Triangulate 3D points from relative pose.
     * 
     * @param pts_prev Points in previous frame
     * @param pts_curr Points in current frame
     * @param R Relative rotation
     * @param t Relative translation
     * @param K Camera intrinsics
     * @param points_3d Output 3D points
     * @return true
     */
    bool triangulate(const std::vector<cv::Point2f>& pts_prev,
                     const std::vector<cv::Point2f>& pts_curr,
                     const cv::Mat& R,
                     const cv::Mat& t,
                     const cv::Mat& K,
                     std::vector<cv::Point3f>& points_3d);
};

} // namespace frontend