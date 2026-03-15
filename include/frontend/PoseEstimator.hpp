#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "Frame.hpp"

namespace frontend {

/**
 * @brief Geometric pose estimation for the visual SLAM frontend.
 *
 * Provides three core capabilities used at different stages of tracking:
 *   1. **Essential-matrix estimation** (2D-2D) — used only during map
 *      initialization, before any 3D points exist.
 *   2. **PnP refinement** (3D-2D) — used during steady-state tracking to
 *      refine the frame pose from MapPoint projections, with RANSAC outlier
 *      rejection.
 *   3. **Triangulation** — converts two-view 2D correspondences into 3D
 *      points in the first camera's frame.
 *
 * Camera intrinsics must be set once via setIntrinsics() before calling
 * estimateRefined().
 */
class PoseEstimator {
public:
    using Ptr = std::shared_ptr<PoseEstimator>;

    PoseEstimator();
    ~PoseEstimator() = default;

    /**
     * @brief Store camera intrinsics (call once during construction).
     * @param K  3x3 camera intrinsic matrix (CV_64F).
     */
    void setIntrinsics(const cv::Mat &K);

    /**
     * @brief Estimate the relative pose between two views via the Essential matrix.
     *
     * Uses cv::findEssentialMat (RANSAC) followed by cv::recoverPose.
     * Only used during initialization when no 3D map exists yet.
     *
     * @param points_prev  2D keypoints in the previous frame.
     * @param points_curr  2D keypoints in the current frame.
     * @param K            3x3 camera intrinsic matrix.
     * @param[out] R       Recovered 3x3 rotation matrix.
     * @param[out] t       Recovered 3x1 translation vector (unit norm).
     * @param[out] mask    Per-correspondence inlier mask from RANSAC.
     * @return true if at least kMinEssentialPoints inliers were recovered.
     */
    bool estimate(const std::vector<cv::Point2f>& points_prev,
                  const std::vector<cv::Point2f>& points_curr,
                  const cv::Mat& K,
                  cv::Mat& R,
                  cv::Mat& t,
                  cv::Mat& mask);

    /**
     * @brief Refine the frame's pose via PnP using its MapPoint associations.
     *
     * Builds 3D-2D correspondences from the frame's MapPoint vector, runs
     * cv::solvePnPRansac with the current pose as initial guess, removes
     * outlier associations, and updates the frame's pose on success.
     *
     * @param frame         The frame whose pose will be refined in-place.
     * @param[out] inlier_count  If non-null, receives the number of PnP inliers.
     * @return true if PnP succeeded with at least kMinPnPCorrespondences inliers.
     */
    bool estimateRefined(Frame::Ptr frame, int *inlier_count = nullptr);

    /**
     * @brief Triangulate 3D points from two-view 2D correspondences.
     *
     * Constructs projection matrices P1 = K*[I|0] and P2 = K*[R|t], calls
     * cv::triangulatePoints, and converts the homogeneous result to 3D.
     * Invalid points (degenerate w) are emitted as NaN.
     *
     * @param pts_prev       2D points in camera 1.
     * @param pts_curr       2D points in camera 2.
     * @param K              3x3 camera intrinsic matrix.
     * @param R              3x3 rotation from camera 1 to camera 2.
     * @param t              3x1 translation from camera 1 to camera 2.
     * @param[out] points_3d Triangulated 3D points in camera 1's frame.
     * @return true if at least one point was triangulated.
     */
    bool triangulate(const std::vector<cv::Point2f> &pts_prev,
                     const std::vector<cv::Point2f> &pts_curr, const cv::Mat &K,
                     const cv::Mat &R, const cv::Mat &t,
                     std::vector<cv::Point3f> &points_3d);

  private:
    cv::Mat K_;            ///< Cached 3x3 camera intrinsic matrix.
    cv::Mat dist_coeffs_;  ///< Distortion coefficients (zeros — undistorted input assumed).

    static constexpr size_t kMinEssentialPoints = 5;      ///< Min inliers for Essential matrix.
    static constexpr size_t kMinPnPCorrespondences = 10;   ///< Min 3D-2D pairs for PnP.
    static constexpr double kRansacConfidence = 0.99;      ///< RANSAC confidence level.
    static constexpr double kEssentialThreshold = 1.0;     ///< Essential matrix RANSAC threshold (pixels).
    static constexpr int kPnPIterations = 500;             ///< solvePnPRansac iteration limit.
    static constexpr float kPnPReprojThreshold = 6.0f;     ///< PnP reprojection error threshold (pixels).
};

} // namespace frontend
