#include "frontend/PoseEstimator.hpp"
#include "dv_slam/utility.hpp"
#include <gtsam/geometry/Pose3.h>
#include <iostream>

namespace frontend {

PoseEstimator::PoseEstimator() {}

void PoseEstimator::setIntrinsics(const cv::Mat &K) {
  K_ = K.clone();
  dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F); // No distortion
}

bool PoseEstimator::estimate(const std::vector<cv::Point2f> &points_prev,
                             const std::vector<cv::Point2f> &points_curr,
                             const cv::Mat &K, cv::Mat &R, cv::Mat &t,
                             cv::Mat &mask) {
  if (points_prev.size() < 5 || points_curr.size() < 5) {
    return false;
  }

  double focal = K.at<double>(0, 0);
  cv::Point2d pp(K.at<double>(0, 2), K.at<double>(1, 2));

  cv::Mat E = cv::findEssentialMat(points_prev, points_curr, focal, pp,
                                   cv::RANSAC, 0.999, 1.0, mask);
  if (E.empty()) {
    return false;
  }

  int inliers = cv::recoverPose(E, points_prev, points_curr, K, R, t, mask);

  if (inliers < 5) {
    return false;
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────
// estimateRefined: PnP pose from 3D MapPoints → 2D keypoints
// ─────────────────────────────────────────────────────────────────
bool PoseEstimator::estimateRefined(Frame::Ptr frame, int *n_inliers) {
  if (K_.empty()) {
    std::cerr
        << "[PoseEstimator] Intrinsics not set! Call setIntrinsics() first."
        << std::endl;
    return false;
  }

  // ── 1. Build 3D-2D correspondences from frame's MapPoint associations ──
  std::vector<cv::Point3f> object_points; // 3D world coordinates
  std::vector<cv::Point2f> image_points;  // 2D pixel coordinates
  std::vector<size_t> kp_indices; // Which keypoint index each pair came from

  const auto &map_points = frame->getMapPoints();
  const auto &keypoints = frame->getKeypoints();

  for (size_t i = 0; i < map_points.size(); i++) {
    if (!map_points[i] || map_points[i]->isBad_)
      continue;

    const Eigen::Vector3d &pos = map_points[i]->position_;
    object_points.emplace_back((float)pos.x(), (float)pos.y(), (float)pos.z());
    image_points.push_back(keypoints[i].pt);
    kp_indices.push_back(i);
  }

  // Need minimum 10 correspondences for robust PnP
  if (object_points.size() < 10) {
    return false;
  }

  // ── 2. Convert current pose (T_w_c) to OpenCV camera-from-world (T_c_w) ──
  gtsam::Pose3 T_w_c = frame->getPose();
  gtsam::Pose3 T_c_w = T_w_c.inverse();

  Eigen::Matrix3d R_eigen = T_c_w.rotation().matrix();
  Eigen::Vector3d t_eigen = T_c_w.translation();

  cv::Mat R_cv(3, 3, CV_64F);
  cv::Mat t_cv(3, 1, CV_64F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      R_cv.at<double>(i, j) = R_eigen(i, j);
    t_cv.at<double>(i) = t_eigen(i);
  }

  cv::Mat rvec;
  cv::Rodrigues(R_cv, rvec);

  // ── 3. Solve PnP with RANSAC ──
  cv::Mat inlier_indices;
  bool success = cv::solvePnPRansac(
      object_points, image_points, K_, dist_coeffs_, rvec, t_cv,
      true, // useExtrinsicGuess — use our initial pose
      200,  // iterationsCount
      4.0,  // reprojectionError threshold (pixels)
      0.99, // confidence
      inlier_indices, cv::SOLVEPNP_ITERATIVE);

  if (!success || inlier_indices.rows < 10) {
    if (n_inliers)
      *n_inliers = inlier_indices.empty() ? 0 : inlier_indices.rows;
    return false;
  }

  if (n_inliers)
    *n_inliers = inlier_indices.rows;

  // ── 4. Remove outlier MapPoint associations ──
  std::set<int> inlier_set;
  for (int i = 0; i < inlier_indices.rows; i++) {
    inlier_set.insert(inlier_indices.at<int>(i));
  }

  for (size_t i = 0; i < kp_indices.size(); i++) {
    if (inlier_set.find((int)i) == inlier_set.end()) {
      // This correspondence was an outlier → remove MapPoint association
      frame->accessMapPoints()[kp_indices[i]] = nullptr;
    }
  }

  // ── 5. Convert refined pose back to GTSAM (world-from-camera) ──
  cv::Mat R_refined;
  cv::Rodrigues(rvec, R_refined);

  // Build T_c_w from OpenCV result
  gtsam::Pose3 T_c_w_refined = cvToGtsam(R_refined, t_cv);

  // Store T_w_c
  frame->setPose(T_c_w_refined.inverse());

  return true;
}

bool PoseEstimator::triangulate(const std::vector<cv::Point2f> &points_prev,
                                const std::vector<cv::Point2f> &points_curr,
                                const cv::Mat &K, const cv::Mat &R,
                                const cv::Mat &t,
                                std::vector<cv::Point3f> &points_3d) {
  cv::Mat T1 = cv::Mat::eye(3, 4, CV_64F);
  cv::Mat T2 = cv::Mat::zeros(3, 4, CV_64F);
  R.copyTo(T2(cv::Rect(0, 0, 3, 3)));
  t.copyTo(T2(cv::Rect(3, 0, 1, 3)));
  cv::Mat P1 = K * T1;
  cv::Mat P2 = K * T2;
  cv::Mat points_4d;
  cv::triangulatePoints(P1, P2, points_prev, points_curr, points_4d);

  points_3d.clear();
  for (int i = 0; i < points_4d.cols; i++) {
    float w = points_4d.at<float>(3, i);
    if (std::abs(w) < 1e-6f)
      continue;
    cv::Point3f point(points_4d.at<float>(0, i) / w,
                      points_4d.at<float>(1, i) / w,
                      points_4d.at<float>(2, i) / w);
    points_3d.push_back(point);
  }
  return true;
}

} // namespace frontend