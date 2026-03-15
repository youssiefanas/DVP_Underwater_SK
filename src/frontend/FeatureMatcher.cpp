#include "frontend/FeatureMatcher.hpp"
#include <iostream>
#include <gtsam/geometry/Point3.h>

namespace frontend {

FeatureMatcher::FeatureMatcher(const std::string &matcher_type,
                               float ratio_thresh)
    : ratio_thresh_(ratio_thresh) {

  if (matcher_type == "NORM_L2") {
    matcher_ = cv::DescriptorMatcher::create("BruteForce");
  } else {
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
  }
}

std::vector<cv::DMatch> FeatureMatcher::match(const cv::Mat &desc1,
                                               const cv::Mat &desc2,
                                               float ratio_thresh) {
    std::vector<cv::DMatch> good_matches;

    if (desc1.empty() || desc2.empty()) {
        return good_matches;
    }

    const float thresh = (ratio_thresh > 0.0f) ? ratio_thresh : ratio_thresh_;

    std::vector<std::vector<cv::DMatch>> knn_matches;

    try {
        matcher_->knnMatch(desc1, desc2, knn_matches, 2);
    } catch (const cv::Exception& e) {
        std::cerr << "[FeatureMatcher] Error during matching: " << e.what() << std::endl;
        return good_matches;
    }

    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i].size() < 2) continue;

        if (knn_matches[i][0].distance < thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    return good_matches;
}

std::vector<cv::DMatch> FeatureMatcher::match(Frame::Ptr frame1, Frame::Ptr frame2) {
    if (!frame1 || !frame2) {
        return {};
    }
    return match(frame1->getDescriptors(), frame2->getDescriptors());
}

int FeatureMatcher::matchByProjection(
    Frame::Ptr frame, const std::vector<MapPoint::Ptr> &map_points,
    const cv::Mat &K, const gtsam::Pose3 &T_w_c, float search_radius) {
  if (!frame || frame->getDescriptors().empty() || frame->getKeypoints().empty() ||
      K.empty()) {
    return 0;
  }
  frame->ensureMapPointVectorSized(frame->getKeypoints().size());

  int matches = 0;
  gtsam::Pose3 T_c_w = T_w_c.inverse();

  double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);
  int img_cols = frame->getImage().cols;
  int img_rows = frame->getImage().rows;

  const float search_r2 = search_radius * search_radius;
  const int kMaxHammingDist = 80; // Absolute threshold
  const auto &keypoints = frame->getKeypoints();
  const cv::Mat &descriptors = frame->getDescriptors();

  for (const auto &mp : map_points) {
    if (!mp || mp->isBad_)
      continue;
    if (mp->descriptor_.empty())
      continue; // Guard: need a descriptor to match

    // 1. Transform to camera frame
    gtsam::Point3 p_cam = T_c_w.transformFrom(gtsam::Point3(mp->position_));
    if (p_cam.z() <= 0.1)
      continue; // Behind camera or too close

    // 2. Project to image
    double u = fx * p_cam.x() / p_cam.z() + cx;
    double v = fy * p_cam.y() / p_cam.z() + cy;

    // 3. Check bounds (with margin)
    if (u < 0 || u >= img_cols || v < 0 || v >= img_rows)
      continue;

    // 4. Find best AND second-best keypoint within search_radius
    int best_idx = -1;
    int best_dist = 256;
    int second_best_dist = 256;

    for (size_t j = 0; j < keypoints.size(); j++) {
      if (j >= frame->accessMapPoints().size()) {
        continue;
      }
      if (frame->accessMapPoints()[j])
        continue; // Already matched

      const cv::KeyPoint &kp = keypoints[j];
      float dx = kp.pt.x - (float)u;
      float dy = kp.pt.y - (float)v;
      if (dx * dx + dy * dy > search_r2)
        continue;

      int dist =
          cv::norm(mp->descriptor_, descriptors.row((int)j), cv::NORM_HAMMING);
      if (dist < best_dist) {
        second_best_dist = best_dist;
        best_dist = dist;
        best_idx = (int)j;
      } else if (dist < second_best_dist) {
        second_best_dist = dist;
      }
    }

    // 5. Accept match: absolute threshold + ratio test vs second best
    if (best_idx >= 0 && best_dist < kMaxHammingDist) {
      // If there's a second candidate, require ratio test
      if (second_best_dist < 256) {
        if ((float)best_dist > 0.9f * (float)second_best_dist)
          continue; // Ambiguous match
      }
      frame->accessMapPoints()[best_idx] = mp;
      matches++;
    }
  }
  return matches;
}

} // namespace frontend
