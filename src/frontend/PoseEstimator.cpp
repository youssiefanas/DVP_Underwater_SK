#include "frontend/PoseEstimator.hpp"
#include "frontend/FixedLandmarkProjectionFactor.hpp"
#include "dv_slam/utility.hpp"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PoseRotationPrior.h>

namespace frontend {

// ─── Construction ────────────────────────────────────────────────

PoseEstimator::PoseEstimator() {}

void PoseEstimator::setIntrinsics(const cv::Mat &K,
                                  const cv::Mat &dist_coeffs) {
  K_ = K.clone();
  dist_coeffs_ = dist_coeffs.clone();
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double s = K.at<double>(0, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);
  gtsam_K_ = std::make_shared<gtsam::Cal3_S2>(fx, fy, s, cx, cy);
}

void PoseEstimator::setAttitudePriorSigmas(double roll_sigma_rad,
                                           double pitch_sigma_rad) {
  roll_sigma_rad_ = roll_sigma_rad;
  pitch_sigma_rad_ = pitch_sigma_rad;
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

bool PoseEstimator::estimateRefined(Frame::Ptr frame, int *inlier_count) {
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

  // Convert refined pose back: T_c_w → T_w_c
  cv::Mat R_refined;
  cv::Rodrigues(rvec, R_refined);
  frame->setPose(cvToPose3d(R_refined, t_cv).inverse());

  return true;
}

// ─── Motion-only Bundle Adjustment ──────────────────────────────

bool PoseEstimator::motionOnlyBA(Frame::Ptr frame, int *inlier_count,
                                 Eigen::Matrix<double, 6, 6> *covariance) {
  if (!frame || !gtsam_K_) {
    if (inlier_count)
      *inlier_count = 0;
    return false;
  }

  // Collect 3D-2D correspondences from surviving inlier MapPoints
  const auto &map_points = frame->getMapPoints();
  const auto &keypoints = frame->getKeypoints();

  std::vector<gtsam::Point3> landmarks;
  std::vector<gtsam::Point2> measurements;
  std::vector<size_t> kp_indices;

  for (size_t i = 0; i < map_points.size() && i < keypoints.size(); i++) {
    if (!map_points[i] || map_points[i]->isBad_)
      continue;
    const Eigen::Vector3d &pos = map_points[i]->position_;
    landmarks.emplace_back(pos.x(), pos.y(), pos.z());
    measurements.emplace_back(keypoints[i].pt.x, keypoints[i].pt.y);
    kp_indices.push_back(i);
  }

  if (landmarks.size() < kMinPnPCorrespondences) {
    if (inlier_count)
      *inlier_count = 0;
    return false;
  }

  // Build factor graph: single Pose3 variable + N projection factors
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  // Convert Eigen::Isometry3d → gtsam::Pose3
  Eigen::Matrix4d mat = frame->getPose().matrix();
  gtsam::Pose3 init_pose(mat);
  initial.insert(X(0), init_pose);

  // Huber robust kernel (k = 1.345 ≈ 95% efficiency for Gaussian)
  auto huber = gtsam::noiseModel::mEstimator::Huber::Create(1.345);
  auto pixel_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  auto robust_noise = gtsam::noiseModel::Robust::Create(huber, pixel_noise);

  for (size_t i = 0; i < landmarks.size(); i++) {
    graph.emplace_shared<FixedLandmarkProjectionFactor>(
        measurements[i], robust_noise, X(0), landmarks[i], gtsam_K_);
  }

  // Optional IMU-derived roll/pitch prior on rotation. Sigmas are in body-frame
  // tangent components (rx, ry, rz) ≈ (pitch about cam-x, yaw about cam-y,
  // roll about cam-z). Yaw is left effectively unconstrained.
  const bool prior_enabled = frame->getAttitudePrior().has_value() &&
                             roll_sigma_rad_ > 0.0 && pitch_sigma_rad_ > 0.0;
  gtsam::SharedNoiseModel rot_prior_noise;
  gtsam::Pose3 rot_prior_pose;
  if (prior_enabled) {
    constexpr double kYawFreeSigma = 1e3;
    gtsam::Vector3 sigmas(pitch_sigma_rad_, kYawFreeSigma, roll_sigma_rad_);
    rot_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    rot_prior_pose =
        gtsam::Pose3(gtsam::Rot3(*frame->getAttitudePrior()),
                     init_pose.translation());
    graph.emplace_shared<gtsam::PoseRotationPrior<gtsam::Pose3>>(
        X(0), rot_prior_pose, rot_prior_noise);
  }

  // Optimize (converges fast from a good PnP init)
  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = 10;
  params.relativeErrorTol = 1e-5;
  params.absoluteErrorTol = 1e-5;

  gtsam::Values result;
  try {
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    result = optimizer.optimize();
  } catch (const std::exception &e) {
    std::cerr << "[PoseEstimator] Motion-only BA failed: " << e.what()
              << std::endl;
    if (inlier_count)
      *inlier_count = 0;
    return false;
  }

  gtsam::Pose3 optimized = result.at<gtsam::Pose3>(X(0));

  // Update frame pose
  Pose3d pose_out = Pose3d::Identity();
  pose_out.matrix() = optimized.matrix();
  frame->setPose(pose_out);

  // Post-BA outlier rejection by reprojection error
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera(optimized, *gtsam_K_);
  const double reproj_thresh_sq =
      static_cast<double>(kPnPReprojThreshold * kPnPReprojThreshold);
  int n_inliers = 0;

  for (size_t i = 0; i < landmarks.size(); i++) {
    try {
      gtsam::Point2 proj = camera.project(landmarks[i]);
      double dx = proj.x() - measurements[i].x();
      double dy = proj.y() - measurements[i].y();
      if (dx * dx + dy * dy > reproj_thresh_sq) {
        frame->accessMapPoints()[kp_indices[i]] = nullptr;
      } else {
        n_inliers++;
      }
    } catch (gtsam::CheiralityException &) {
      frame->accessMapPoints()[kp_indices[i]] = nullptr;
    }
  }

  if (inlier_count)
    *inlier_count = n_inliers;

  if (n_inliers < static_cast<int>(kMinPnPCorrespondences))
    return false;

  // Extract covariance from GTSAM Marginals
  if (covariance) {
    try {
      // Rebuild graph with non-robust noise for meaningful covariance
      gtsam::NonlinearFactorGraph cov_graph;
      for (size_t i = 0; i < landmarks.size(); i++) {
        if (!frame->getMapPoints()[kp_indices[i]])
          continue; // skip rejected outliers
        cov_graph.emplace_shared<FixedLandmarkProjectionFactor>(
            measurements[i], pixel_noise, X(0), landmarks[i], gtsam_K_);
      }
      if (prior_enabled) {
        cov_graph.emplace_shared<gtsam::PoseRotationPrior<gtsam::Pose3>>(
            X(0), rot_prior_pose, rot_prior_noise);
      }
      gtsam::Marginals marginals(cov_graph, result,
                                 gtsam::Marginals::CHOLESKY);
      *covariance = marginals.marginalCovariance(X(0));
    } catch (const std::exception &) {
      // Fallback: default diagonal covariance
      *covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
    }
  }

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
