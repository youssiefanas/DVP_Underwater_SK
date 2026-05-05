#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <gtsam/geometry/Cal3_S2.h>

#include "Frame.hpp"
#include "Pose3d.hpp"

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
     * @param dist_coeffs  Distortion coefficients (CV_64F).
     */
    void setIntrinsics(const cv::Mat &K, const cv::Mat &dist_coeffs);

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
     * @brief Refine the frame's pose via PnP RANSAC and remove outlier
     *        MapPoint associations.
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

    /// Optional translation-only prior used inside motionOnlyBA() to inject
    /// a metric DVL-derived displacement.
    struct DvlTranslationPrior {
      Pose3d T_w_prev{Pose3d::Identity()};        ///< Previous (frozen) world pose.
      Eigen::Vector3d dp_world{Eigen::Vector3d::Zero()}; ///< World-frame displacement.
      Eigen::Matrix3d cov_world{Eigen::Matrix3d::Identity() * 1e-2}; ///< 3x3 covariance.
      bool valid = false;
    };

    /**
     * @brief Motion-only Bundle Adjustment: optimize the frame's pose while
     *        holding all MapPoint positions fixed.
     *
     * Builds a GTSAM factor graph with one Pose3 variable and one
     * FixedLandmarkProjectionFactor per inlier MapPoint. Uses a Huber
     * robust kernel to down-weight remaining outliers. Extracts the 6x6
     * pose covariance from GTSAM Marginals.
     *
     * If @p dvl_prior is non-null and valid, an additional translation-only
     * prior factor pulls the pose's translation toward
     * `T_w_prev.translation() + dp_world` with the supplied covariance.
     * This injects a metric scale signal from DVL while leaving rotation
     * to the visual evidence.
     *
     * Call after PnP RANSAC has set an initial pose and removed gross
     * outliers.
     *
     * @param frame         Frame with pose initial guess and MapPoint associations.
     * @param[out] inlier_count  Number of inliers after post-BA outlier rejection.
     * @param[out] covariance    6x6 pose covariance [rot(3), trans(3)].
     * @param dvl_prior     Optional DVL translation prior (nullptr → vision only).
     * @return true if optimization succeeded.
     */
    bool motionOnlyBA(Frame::Ptr frame, int *inlier_count = nullptr,
                      Eigen::Matrix<double, 6, 6> *covariance = nullptr,
                      const DvlTranslationPrior *dvl_prior = nullptr);

  private:
    cv::Mat K_;            ///< Cached 3x3 camera intrinsic matrix.
    cv::Mat dist_coeffs_;  ///< Distortion coefficients (zeros — undistorted input assumed).
    std::shared_ptr<gtsam::Cal3_S2> gtsam_K_; ///< GTSAM calibration (created in setIntrinsics).

    static constexpr size_t kMinEssentialPoints = 5;      ///< Min inliers for Essential matrix.
    static constexpr size_t kMinPnPCorrespondences = 10;   ///< Min 3D-2D pairs for PnP.
    static constexpr double kRansacConfidence = 0.99;      ///< RANSAC confidence level.
    static constexpr double kEssentialThreshold = 1.0;     ///< Essential matrix RANSAC threshold (pixels).
    static constexpr int kPnPIterations = 500;             ///< solvePnPRansac iteration limit.
    static constexpr float kPnPReprojThreshold = 3.5f;     ///< PnP reprojection error threshold (pixels).
};

} // namespace frontend
