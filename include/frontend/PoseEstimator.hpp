#pragma once

#include "Frame.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

namespace frontend {

class PoseEstimator {
public:
    using Ptr = std::shared_ptr<PoseEstimator>;

    PoseEstimator();
    ~PoseEstimator() = default;
    /**
     * @brief Store camera intrinsics for PnP.
     *        Must be called once during VisualFrontend construction.
     */
    void setIntrinsics(const cv::Mat &K);

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
     * @brief Refine pose using PnP with MapPoint observations (3D-2D).
     *        Reads the frame's MapPoint associations, builds 3D-2D
     * correspondences, runs solvePnPRansac with the frame's current pose as
     * initial guess. On success, updates the frame's pose and removes outlier
     * associations.
     *
     * @param frame     Frame with MapPoint associations and an initial pose
     * guess.
     * @param n_inliers Output: number of PnP inliers (optional, can be
     * nullptr).
     * @return true if PnP succeeded with enough inliers.
     */
    bool estimateRefined(Frame::Ptr frame, int *n_inliers = nullptr);

    /**
     * @brief Triangulate 3D points from two sets of 2D correspondences.
     */
    bool triangulate(const std::vector<cv::Point2f> &pts_prev,
                     const std::vector<cv::Point2f> &pts_curr, const cv::Mat &K,
                     const cv::Mat &R, const cv::Mat &t,
                     std::vector<cv::Point3f> &points_3d);

  private:
    cv::Mat K_;           // Camera intrinsics (set once)
    cv::Mat dist_coeffs_; // Distortion (empty = no distortion)
};

} // namespace frontend