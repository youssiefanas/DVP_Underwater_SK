#include "frontend/PoseEstimator.hpp"
#include "dv_slam/utility.hpp"
#include <gtsam/geometry/Pose3.h>
#include <iostream>
#include <set>

namespace frontend {

PoseEstimator::PoseEstimator() {}

void PoseEstimator::setIntrinsics(const cv::Mat &K) {
  K_ = K.clone();
  dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
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
  if (E.empty())
    return false;

  int inliers = cv::recoverPose(E, points_prev, points_curr, K, R, t, mask);
  return (inliers >= 5);
}

bool PoseEstimator::estimateRefined(Frame::Ptr frame, int *inlier_count) {
  if (K_.empty()) {
    std::cerr << "[PoseEstimator] Intrinsics not set!" << std::endl;
    return false;
  }

  // Build 3D-2D correspondences from frame's MapPoint associations
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  std::vector<size_t> keypoint_indices;

  const auto &map_points = frame->getMapPoints();
  const auto &keypoints = frame->getKeypoints();

  for (size_t i = 0; i < map_points.size(); i++) {
    if (!map_points[i] || map_points[i]->isBad_)
      continue;
    if (map_points[i]->descriptor_.empty())
      continue;

    const Eigen::Vector3d &position = map_points[i]->position_;
    object_points.emplace_back((float)position.x(), (float)position.y(),
                               (float)position.z());
    image_points.push_back(keypoints[i].pt);
    keypoint_indices.push_back(i);
  }

  if (object_points.size() < 10) {
    if (inlier_count)
      *inlier_count = 0;
    return false;
  }

  // Convert current pose T_w_c to OpenCV T_c_w (camera-from-world)
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

  // solvePnPRansac with initial guess
  cv::Mat inlier_indices;
  bool success = cv::solvePnPRansac(object_points, image_points, K_,
                                    dist_coeffs_, rvec, t_cv,
                                    true, // useExtrinsicGuess
                                    200,  // iterations
                                    4.0,  // reprojection threshold (pixels)
                                    0.99, // confidence
                                    inlier_indices, cv::SOLVEPNP_ITERATIVE);

  int n_inliers = inlier_indices.empty() ? 0 : inlier_indices.rows;
  if (inlier_count)
    *inlier_count = n_inliers;

  if (!success || n_inliers < 10) {
    return false;
  }

  // Remove outlier MapPoint associations
  std::set<int> inlier_set;
  for (int i = 0; i < inlier_indices.rows; i++) {
    inlier_set.insert(inlier_indices.at<int>(i));
  }
  for (size_t i = 0; i < keypoint_indices.size(); i++) {
    if (inlier_set.find((int)i) == inlier_set.end()) {
      frame->accessMapPoints()[keypoint_indices[i]] = nullptr;
    }
  }

  // Convert refined pose back: T_c_w → T_w_c
  cv::Mat R_refined;
  cv::Rodrigues(rvec, R_refined);
  gtsam::Pose3 T_c_w_refined = cvToGtsam(R_refined, t_cv);
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
    points_3d.emplace_back(points_4d.at<float>(0, i) / w,
                           points_4d.at<float>(1, i) / w,
                           points_4d.at<float>(2, i) / w);
  }
  return true;
}

} // namespace frontend