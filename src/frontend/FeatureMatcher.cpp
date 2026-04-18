#include "frontend/FeatureMatcher.hpp"
#include <cstdint>
#include <iostream>

namespace frontend {

// ─── Spatial grid for fast radius lookup ─────────────────────────
namespace {

// Hamming distance for 32-byte ORB descriptors: XOR + popcount over 4×uint64.
// Avoids cv::Mat header allocation and OpenCV's generic norm dispatch in the
// projection-matching inner loop.
static inline int hamming32(const uchar *a, const uchar *b) {
  const uint64_t *pa = reinterpret_cast<const uint64_t *>(a);
  const uint64_t *pb = reinterpret_cast<const uint64_t *>(b);
  return __builtin_popcountll(pa[0] ^ pb[0]) +
         __builtin_popcountll(pa[1] ^ pb[1]) +
         __builtin_popcountll(pa[2] ^ pb[2]) +
         __builtin_popcountll(pa[3] ^ pb[3]);
}

class KeypointGrid {
public:
  KeypointGrid(const std::vector<cv::KeyPoint> &keypoints,
               const std::vector<MapPoint::Ptr> &map_points, int img_cols,
               int img_rows, float cell_size)
      : cell_size_(cell_size),
        cols_(std::max(1, static_cast<int>(std::ceil(img_cols / cell_size)))),
        rows_(std::max(1, static_cast<int>(std::ceil(img_rows / cell_size)))),
        cells_(cols_ * rows_) {
    for (size_t j = 0; j < keypoints.size(); j++) {
      if (j < map_points.size() && map_points[j])
        continue; // Already matched — skip
      int cx = static_cast<int>(keypoints[j].pt.x / cell_size_);
      int cy = static_cast<int>(keypoints[j].pt.y / cell_size_);
      cx = std::clamp(cx, 0, cols_ - 1);
      cy = std::clamp(cy, 0, rows_ - 1);
      cells_[cy * cols_ + cx].push_back(static_cast<int>(j));
    }
  }

  // Return indices of keypoints in cells overlapping the search radius
  void getCandidates(float u, float v, float radius,
                     std::vector<int> &out) const {
    int min_cx = std::max(0, static_cast<int>((u - radius) / cell_size_));
    int max_cx = std::min(cols_ - 1, static_cast<int>((u + radius) / cell_size_));
    int min_cy = std::max(0, static_cast<int>((v - radius) / cell_size_));
    int max_cy = std::min(rows_ - 1, static_cast<int>((v + radius) / cell_size_));

    for (int cy = min_cy; cy <= max_cy; cy++) {
      for (int cx = min_cx; cx <= max_cx; cx++) {
        const auto &cell = cells_[cy * cols_ + cx];
        out.insert(out.end(), cell.begin(), cell.end());
      }
    }
  }

private:
  float cell_size_;
  int cols_, rows_;
  std::vector<std::vector<int>> cells_;
};

} // anonymous namespace

FeatureMatcher::FeatureMatcher(const std::string &matcher_type,
                               float ratio_thresh)
    : ratio_thresh_(ratio_thresh) {

  if (matcher_type == "NORM_L2") {
    matcher_ = cv::DescriptorMatcher::create("BruteForce");
  } else {
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
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
  } catch (const cv::Exception &e) {
    std::cerr << "[FeatureMatcher] Error during matching: " << e.what()
              << std::endl;
    return good_matches;
  }

  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i].size() < 2)
      continue;

    if (knn_matches[i][0].distance < thresh * knn_matches[i][1].distance) {
      good_matches.push_back(knn_matches[i][0]);
    }
  }
  return good_matches;
}

std::vector<cv::DMatch> FeatureMatcher::match(Frame::Ptr frame1,
                                              Frame::Ptr frame2) {
  if (!frame1 || !frame2) {
    return {};
  }
  return match(frame1->getDescriptors(), frame2->getDescriptors());
}

int FeatureMatcher::matchByProjection(
    Frame::Ptr frame, const std::vector<MapPoint::Ptr> &map_points,
    const cv::Mat &K, const Pose3d &T_w_c, float search_radius) {
  if (!frame || frame->getDescriptors().empty() || frame->getKeypoints().empty() ||
      K.empty()) {
    return 0;
  }
  frame->ensureMapPointVectorSized(frame->getKeypoints().size());

  int matches = 0;
  Pose3d T_c_w = T_w_c.inverse();

  double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);
  int img_cols = frame->getImage().cols;
  int img_rows = frame->getImage().rows;

  const float search_r2 = search_radius * search_radius;
  const int kMaxHammingDist = 80; // Absolute threshold
  const auto &keypoints = frame->getKeypoints();
  const cv::Mat &descriptors = frame->getDescriptors();

  // hamming32() assumes 32-byte ORB descriptors; bail out if that ever changes.
  if (descriptors.cols != 32 || descriptors.type() != CV_8U ||
      !descriptors.isContinuous())
    return 0;

  // Build spatial grid once — cell size = search_radius so a radius query
  // touches at most 3×3 = 9 cells instead of all keypoints.
  KeypointGrid grid(keypoints, frame->accessMapPoints(), img_cols, img_rows,
                    search_radius);
  std::vector<int> candidates;

  for (const auto &mp : map_points) {
    if (!mp || mp->isBad_)
      continue;
    if (mp->descriptor_.empty() || mp->descriptor_.cols != 32)
      continue; // Guard: need a 32-byte descriptor to match

    // 1. Transform to camera frame
    Eigen::Vector3d p_cam = T_c_w * mp->position_;
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

    candidates.clear();
    grid.getCandidates(static_cast<float>(u), static_cast<float>(v),
                       search_radius, candidates);

    const uchar *mp_desc = mp->descriptor_.ptr<uchar>(0);

    for (int j : candidates) {
      if (frame->accessMapPoints()[j])
        continue; // Already matched by an earlier map point in this call

      const cv::KeyPoint &kp = keypoints[j];
      float dx = kp.pt.x - static_cast<float>(u);
      float dy = kp.pt.y - static_cast<float>(v);
      if (dx * dx + dy * dy > search_r2)
        continue;

      int dist = hamming32(mp_desc, descriptors.ptr<uchar>(j));
      if (dist < best_dist) {
        second_best_dist = best_dist;
        best_dist = dist;
        best_idx = j;
      } else if (dist < second_best_dist) {
        second_best_dist = dist;
      }
    }

    // 5. Accept match: absolute threshold + ratio test vs second best
    if (best_idx >= 0 && best_dist < kMaxHammingDist) {
      // If there's a second candidate, require ratio test
      if (second_best_dist < 256) {
        if (static_cast<float>(best_dist) > 0.9f * static_cast<float>(second_best_dist))
          continue; // Ambiguous match
      }
      frame->accessMapPoints()[best_idx] = mp;
      matches++;
    }
  }
  return matches;
}

} // namespace frontend
