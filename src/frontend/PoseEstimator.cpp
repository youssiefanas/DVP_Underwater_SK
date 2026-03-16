#include "frontend/PoseEstimator.hpp"
#include "dv_slam/utility.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>

namespace frontend {

// ─── Construction ────────────────────────────────────────────────

PoseEstimator::PoseEstimator() {}

void PoseEstimator::setIntrinsics(const cv::Mat &K,
                                  const cv::Mat &dist_coeffs) {
  K_ = K.clone();
  dist_coeffs_ = dist_coeffs.clone();
}

// ─── 2D-2D: Essential matrix (initialization only) ──────────────

bool PoseEstimator::estimate(const std::vector<cv::Point2f> &points_prev,
                             const std::vector<cv::Point2f> &points_curr,
                             const cv::Mat &K, cv::Mat &R, cv::Mat &t,
                             cv::Mat &mask) {
  if (points_prev.size() < kMinEssentialPoints ||
      points_curr.size() < kMinEssentialPoints)
    return false;

  double focal = K.at<double>(0, 0);
  cv::Point2d pp(K.at<double>(0, 2), K.at<double>(1, 2));

  cv::Mat E = cv::findEssentialMat(points_prev, points_curr, focal, pp,
                                   cv::RANSAC, kRansacConfidence,
                                   kEssentialThreshold, mask);
  if (E.empty())
    return false;

  int inliers = cv::recoverPose(E, points_prev, points_curr, K, R, t, mask);
  return inliers >= static_cast<int>(kMinEssentialPoints);
}

// ─── 3D-2D: PnP refinement (tracking) ──────────────────────────

bool PoseEstimator::estimateRefined(Frame::Ptr frame, int *inlier_count,
                                    Eigen::Matrix<double, 6, 6> *covariance) {
  if (!frame) {
    if (inlier_count)
      *inlier_count = 0;
    return false;
  }
  if (K_.empty() || dist_coeffs_.empty()) {
    std::cerr << "[PoseEstimator] Intrinsics not set!" << std::endl;
    return false;
  }

  // Build 3D-2D correspondences from frame's MapPoint associations
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  std::vector<size_t> keypoint_indices;

  const auto &map_points = frame->getMapPoints();
  const auto &keypoints = frame->getKeypoints();

  for (size_t i = 0; i < map_points.size() && i < keypoints.size(); i++) {
    if (!map_points[i] || map_points[i]->isBad_)
      continue;

    const Eigen::Vector3d &pos = map_points[i]->position_;
    object_points.emplace_back(static_cast<float>(pos.x()),
                               static_cast<float>(pos.y()),
                               static_cast<float>(pos.z()));
    image_points.push_back(keypoints[i].pt);
    keypoint_indices.push_back(i);
  }

  if (object_points.size() < kMinPnPCorrespondences) {
    if (inlier_count)
      *inlier_count = 0;
    return false;
  }

  // Convert current pose T_w_c to OpenCV T_c_w (camera-from-world)
  Pose3d T_c_w = frame->getPose().inverse();
  cv::Mat R_cv, t_cv;
  pose3dToCv(T_c_w, R_cv, t_cv);

  cv::Mat rvec;
  cv::Rodrigues(R_cv, rvec);

  cv::Mat inlier_indices;
  bool success = cv::solvePnPRansac(
      object_points, image_points, K_, dist_coeffs_, rvec, t_cv,
      true,                   // useExtrinsicGuess
      kPnPIterations,         // iterations
      kPnPReprojThreshold,    // reprojection threshold (pixels)
      kRansacConfidence,      // confidence
      inlier_indices, cv::SOLVEPNP_ITERATIVE);

  int n_inliers = inlier_indices.empty() ? 0 : inlier_indices.rows;
  if (inlier_count)
    *inlier_count = n_inliers;

  if (!success || n_inliers < static_cast<int>(kMinPnPCorrespondences))
    return false;

  // Remove outlier MapPoint associations
  std::unordered_set<int> inlier_set;
  inlier_set.reserve(n_inliers);
  for (int i = 0; i < inlier_indices.rows; i++) {
    inlier_set.insert(inlier_indices.at<int>(i));
  }
  for (size_t i = 0; i < keypoint_indices.size(); i++) {
    if (inlier_set.find(static_cast<int>(i)) == inlier_set.end()) {
      frame->accessMapPoints()[keypoint_indices[i]] = nullptr;
    }
  }

  if (covariance) {
    std::vector<cv::Point3f> inlier_obj;
    std::vector<cv::Point2f> inlier_img;
    inlier_obj.reserve(n_inliers);
    inlier_img.reserve(n_inliers);
    for (int i = 0; i < inlier_indices.rows; i++) {
      int idx = inlier_indices.at<int>(i);
      inlier_obj.push_back(object_points[idx]);
      inlier_img.push_back(image_points[idx]);
    }
    if (n_inliers >= 6) {
      std::vector<cv::Point2f> projected;
      cv::Mat full_jacobian; // 2N x (3+3+2+2+DistCoeffs)
      cv::projectPoints(inlier_obj, rvec, t_cv, K_, dist_coeffs_, projected,
                        full_jacobian);
      // jacobian matrix of derivatives of image points with respect to
      // components of the rotation vector (3), translation vector (3), focal
      // lengths (2), coordinates of the principal point (2), and the distortion
      // coefficients
      cv::Mat J_pose = full_jacobian(cv::Range::all(), cv::Range(0, 6));
      double sse = 0.0;
      for (int i = 0; i < n_inliers; i++) {
        double dx = inlier_img[i].x - projected[i].x;
        double dy = inlier_img[i].y - projected[i].y;
        sse += dx * dx + dy * dy;
      }
      // sigma2 is the variance of the reprojection error, estimated from the
      // residuals. The denominator is the degrees of freedom: 2N (observations)
      // - 6 (pose parameters).
      double sigma2 = sse / std::max(1, 2 * n_inliers - 6);
      cv::Mat JtJ = J_pose.t() * J_pose;
      cv::Mat JtJ_inv;
      cv::invert(JtJ, JtJ_inv, cv::DECOMP_SVD);
      cv::Mat cov_cv = sigma2 * JtJ_inv;
      for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
          (*covariance)(i, j) = cov_cv.at<double>(i, j);
      *covariance = (*covariance + covariance->transpose()) / 2.0;

      // Clamp eigenvalues to preserve positive-definiteness
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(
          *covariance);
      Eigen::Matrix<double, 6, 1> clamped_evals =
          eig.eigenvalues().cwiseMax(1e-6);
      *covariance = eig.eigenvectors() * clamped_evals.asDiagonal() *
                    eig.eigenvectors().transpose();
    } else {
      *covariance = Eigen::Matrix<double, 6, 6>::Zero();
      covariance->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.01;
      covariance->block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.01;
      std::cerr << "[PoseEstimator] Warning: Not enough inliers to estimate "
                   "covariance. Returning default diagonal."
                << std::endl;
    }
  }

  // Convert refined pose back: T_c_w → T_w_c
  cv::Mat R_refined;
  cv::Rodrigues(rvec, R_refined);
  frame->setPose(cvToPose3d(R_refined, t_cv).inverse());

  return true;
}

// ─── Triangulation ──────────────────────────────────────────────

bool PoseEstimator::triangulate(const std::vector<cv::Point2f> &points_prev,
                                const std::vector<cv::Point2f> &points_curr,
                                const cv::Mat &K, const cv::Mat &R,
                                const cv::Mat &t,
                                std::vector<cv::Point3f> &points_3d) {
  points_3d.clear();
  if (points_prev.empty() || points_prev.size() != points_curr.size())
    return false;

  // P1 = K * [I | 0],  P2 = K * [R | t]
  cv::Mat T1 = cv::Mat::eye(3, 4, CV_64F);
  cv::Mat T2 = cv::Mat::zeros(3, 4, CV_64F);
  R.copyTo(T2(cv::Rect(0, 0, 3, 3)));
  t.copyTo(T2(cv::Rect(3, 0, 1, 3)));
  cv::Mat P1 = K * T1;
  cv::Mat P2 = K * T2;

  cv::Mat points_4d;
  cv::triangulatePoints(P1, P2, points_prev, points_curr, points_4d);
  if (points_4d.empty() || points_4d.rows != 4)
    return false;

  points_3d.reserve(points_4d.cols);
  constexpr float nan = std::numeric_limits<float>::quiet_NaN();

  for (int i = 0; i < points_4d.cols; i++) {
    double x, y, z, w;
    if (points_4d.type() == CV_64F) {
      x = points_4d.at<double>(0, i);
      y = points_4d.at<double>(1, i);
      z = points_4d.at<double>(2, i);
      w = points_4d.at<double>(3, i);
    } else {
      x = points_4d.at<float>(0, i);
      y = points_4d.at<float>(1, i);
      z = points_4d.at<float>(2, i);
      w = points_4d.at<float>(3, i);
    }

    if (!std::isfinite(w) || std::abs(w) < 1e-9) {
      points_3d.emplace_back(nan, nan, nan);
      continue;
    }

    x /= w;
    y /= w;
    z /= w;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      points_3d.emplace_back(nan, nan, nan);
      continue;
    }
    points_3d.emplace_back(static_cast<float>(x), static_cast<float>(y),
                           static_cast<float>(z));
  }
  return !points_3d.empty();
}

} // namespace frontend
