#include "frontend/FeatureMatcher.hpp"
#include <iostream>
#include <gtsam/geometry/Point3.h>

namespace frontend {

FeatureMatcher::FeatureMatcher(const std::string& matcher_type, float ratio_thresh) 
    : ratio_thresh_(ratio_thresh) {
    
    if (matcher_type == "NORM_L2") {
        matcher_ = cv::DescriptorMatcher::create("BruteForce");
    } else {
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
    }
}

std::vector<cv::DMatch> FeatureMatcher::match(Frame::Ptr frame1, Frame::Ptr frame2) {
    std::vector<cv::DMatch> good_matches;
    
    // Check if frames exist and have descriptors
    if (!frame1 || !frame2 || frame1->getDescriptors().empty() || frame2->getDescriptors().empty()) {
        return good_matches;
    }

    std::vector<std::vector<cv::DMatch>> knn_matches;
    
    // k = 2 for Ratio Test
    try {
        matcher_->knnMatch(frame1->getDescriptors(), frame2->getDescriptors(), knn_matches, 2);
    } catch (const cv::Exception& e) {
        std::cerr << "[FeatureMatcher] Error during matching: " << e.what() << std::endl;
        return good_matches;
    }

    // Apply Lowe's Ratio Test
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i].size() < 2) continue;
        
        if (knn_matches[i][0].distance < ratio_thresh_ * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    return good_matches;
}

int FeatureMatcher::matchByProjection(
    Frame::Ptr frame, const std::vector<MapPoint::Ptr> &map_points,
    const cv::Mat &K, const gtsam::Pose3 &T_w_c, float search_radius) {
  int matches = 0;
  gtsam::Pose3 T_c_w = T_w_c.inverse();

  double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);

  for (const auto &mp : map_points) {
    if (!mp || mp->isBad_)
      continue;

    // 1. Transform to camera frame
    gtsam::Point3 p_cam = T_c_w.transformFrom(gtsam::Point3(mp->position_));
    if (p_cam.z() <= 0.0)
      continue; // Behind camera

    // 2. Project to image
    double u = fx * p_cam.x() / p_cam.z() + cx;
    double v = fy * p_cam.y() / p_cam.z() + cy;

    // 3. Check bounds
    if (u < 0 || u >= frame->getImage().cols || v < 0 ||
        v >= frame->getImage().rows)
      continue;

    // 4. Find best keypoint within search_radius
    int best_idx = -1;
    int best_dist = 256; // Max Hamming distance for ORB
    for (size_t j = 0; j < frame->getKeypoints().size(); j++) {
      if (frame->accessMapPoints()[j])
        continue; // Already matched

      const cv::KeyPoint &kp = frame->getKeypoints()[j];
      float dx = kp.pt.x - (float)u;
      float dy = kp.pt.y - (float)v;
      if (dx * dx + dy * dy > search_radius * search_radius)
        continue;

      int dist = cv::norm(mp->descriptor_, frame->getDescriptors().row((int)j),
                          cv::NORM_HAMMING);
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = (int)j;
      }
    }

    // 5. Accept match if good enough
    if (best_idx >= 0 && best_dist < 50) { // Hamming threshold
      frame->accessMapPoints()[best_idx] = mp;
      matches++;
    }
  }
  return matches;
}
}