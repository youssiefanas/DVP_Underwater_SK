#include "frontend/VisualFrontend.hpp"
#include "dv_slam/utility.hpp"
#include <cmath>
#include <iostream>

namespace frontend {

VisualFrontend::VisualFrontend(const ORBParams &params, const cv::Mat &K)
    : stage_(Stage::NO_IMAGES_YET) {
  feature_extractor_ = std::make_shared<FeatureExtractor>(params);
  viewer_ = std::make_shared<Viewer>();
  feature_matcher_ = std::make_shared<FeatureMatcher>(params.matcher_type);
  pose_estimator_ = std::make_shared<PoseEstimator>();
  K_ = K.clone();
  map_ = std::make_shared<Map>();
}

// ─────────────────────────────────────────────────────────────────
// handleImage: entry point — unchanged structure
// ─────────────────────────────────────────────────────────────────
bool VisualFrontend::handleImage(const cv::Mat &gray_image, double timestamp) {
  Frame::Ptr frame = Frame::createFrame(gray_image, timestamp);
  extractFeatures(frame);
  bool success = process(frame);

  if (viewer_) {
    cv::Mat img_out;
    cv::drawKeypoints(frame->getImage(), frame->getKeypoints(), img_out);
    const auto &points = frame->getKeypoints();
    for (const auto &p : points) {
      cv::circle(img_out, p.pt, 2, cv::Scalar(0, 255, 0), -1);
    }
    viewer_->show(img_out);
  }
  return success;
}

void VisualFrontend::extractFeatures(Frame::Ptr frame) {
  feature_extractor_->extract(*frame);
}

// ─────────────────────────────────────────────────────────────────
// process: STATE MACHINE
// ─────────────────────────────────────────────────────────────────
bool VisualFrontend::process(Frame::Ptr current_frame) {
  current_frame_ = current_frame;

  switch (stage_) {

  case Stage::NO_IMAGES_YET: {
    // First frame → set identity pose, create first KeyFrame
    current_frame->setPose(gtsam::Pose3()); // Identity

    last_keyframe_ = KeyFrame::create(current_frame);
    reference_keyframe_ = last_keyframe_;
    map_->addKeyFrame(last_keyframe_);

    last_frame_ = current_frame;
    frames_since_last_kf_ = 0;
    stage_ = Stage::INITIALIZING;

    std::cout << "[Frontend] First frame → KF " << last_keyframe_->getId()
              << " (pose key " << gtsam::Symbol(last_keyframe_->getPoseKey())
              << ")" << std::endl;
    return false;
  }

  case Stage::INITIALIZING: {
    bool ok = tryInitialize(current_frame);
    if (ok) {
      stage_ = Stage::TRACKING;
      std::cout << "[Frontend] Initialization succeeded → TRACKING"
                << std::endl;
    }
    last_frame_ = current_frame;
    frames_since_last_kf_++;
    return ok;
  }

  case Stage::TRACKING: {
    bool ok = track(current_frame);
    last_frame_ = current_frame;
    frames_since_last_kf_++;
    return ok;
  }

  } // end switch

  return false;
}

// ─────────────────────────────────────────────────────────────────
// tryInitialize: Essential matrix between first KF and current frame
// ─────────────────────────────────────────────────────────────────
bool VisualFrontend::tryInitialize(Frame::Ptr current_frame) {
  // Match against the first KeyFrame's underlying frame data
  std::vector<cv::DMatch> matches =
      feature_matcher_->match(last_frame_, current_frame);
  if (matches.size() < 20) {
    std::cout << "[Frontend Init] Not enough matches: " << matches.size()
              << std::endl;
    return false;
  }

  // Extract matched 2D points
  std::vector<cv::Point2f> pts_prev, pts_curr;
  std::vector<cv::DMatch> matches_used;
  for (const auto &m : matches) {
    pts_prev.push_back(last_frame_->getKeypoints()[m.queryIdx].pt);
    pts_curr.push_back(current_frame->getKeypoints()[m.trainIdx].pt);
    matches_used.push_back(m);
  }

  // Essential matrix + pose recovery
  cv::Mat R, t, mask;
  if (!pose_estimator_->estimate(pts_prev, pts_curr, K_, R, t, mask))
    return false;

  // Collect inliers
  std::vector<cv::Point2f> inliers_prev, inliers_curr;
  std::vector<cv::DMatch> inliers_matches;
  for (int i = 0; i < mask.rows; i++) {
    if (mask.at<uchar>(i) == 1) {
      inliers_prev.push_back(pts_prev[i]);
      inliers_curr.push_back(pts_curr[i]);
      inliers_matches.push_back(matches_used[i]);
    }
  }
  if ((int)inliers_prev.size() < 20)
    return false;

  // Set pose
  gtsam::Pose3 T_last_curr = cvToGtsam(R, t);
  gtsam::Pose3 T_w_prev = last_keyframe_->getPose();
  gtsam::Pose3 T_w_curr = T_w_prev * T_last_curr;
  current_frame->setPose(T_w_curr);

  // Check baseline — don't initialize if too small
  double baseline = T_last_curr.translation().norm();
  if (baseline < kMinBaseline) {
    std::cout << "[Frontend Init] Baseline too small: " << baseline
              << std::endl;
    return false;
  }

  // Create second KeyFrame
  KeyFrame::Ptr curr_kf = KeyFrame::create(current_frame);
  map_->addKeyFrame(curr_kf);

  // Triangulate MapPoints between the two KeyFrames
  triangulateNewPoints(last_keyframe_, curr_kf);

  // Update covisibility for both
  last_keyframe_->updateCovisibility();
  curr_kf->updateCovisibility();

  // ──────────────────────────────────────────────────────
  // CRITICAL: Sync MapPoints back into the current Frame
  // ──────────────────────────────────────────────────────
  curr_kf->syncMapPointsToFrame(current_frame);

  // Update state
  last_keyframe_ = curr_kf;
  reference_keyframe_ = curr_kf;
  frames_since_last_kf_ = 0;

  std::cout << "[Frontend Init] KF " << curr_kf->getId()
            << " | baseline=" << baseline
            << " | MapPoints=" << map_->numMapPoints() << std::endl;
  return true;
}
// ─────────────────────────────────────────────────────────────────
// track: main tracking loop (Essential-based for now; PnP in Step 5)
// ─────────────────────────────────────────────────────────────────
bool VisualFrontend::track(Frame::Ptr current_frame) {
  // ── Step 1: Match current frame against last frame ──
  std::vector<cv::DMatch> matches =
      feature_matcher_->match(last_frame_, current_frame);
  if (matches.size() < 20) {
    std::cout << "[Frontend] Not enough matches: " << matches.size()
              << std::endl;
    return false;
  }

  std::vector<cv::Point2f> pts_prev, pts_curr;
  std::vector<cv::DMatch> matches_used;
  for (const auto &m : matches) {
    pts_prev.push_back(last_frame_->getKeypoints()[m.queryIdx].pt);
    pts_curr.push_back(current_frame->getKeypoints()[m.trainIdx].pt);
    matches_used.push_back(m);
  }

  // ── Step 2: Essential matrix pose estimation ──
  cv::Mat R, t, mask;
  if (!pose_estimator_->estimate(pts_prev, pts_curr, K_, R, t, mask)) {
    std::cout << "[Frontend] Pose estimation failed." << std::endl;
    return false;
  }

  // Inlier filtering
  std::vector<cv::Point2f> inliers_prev, inliers_curr;
  std::vector<cv::DMatch> inliers_matches;
  int inliers_count = 0;
  for (int i = 0; i < mask.rows; i++) {
    if (mask.at<uchar>(i) == 1) {
      inliers_count++;
      inliers_prev.push_back(pts_prev[i]);
      inliers_curr.push_back(pts_curr[i]);
      inliers_matches.push_back(matches_used[i]);
    }
  }
  if (inliers_count < 20) {
    std::cout << "[Frontend] Not enough inliers: " << inliers_count
              << std::endl;
    return false;
  }

  // ── Step 3: Compute world pose ──
  gtsam::Pose3 T_last_curr = cvToGtsam(R, t);
  gtsam::Pose3 T_w_last = last_frame_->getPose();
  gtsam::Pose3 T_w_curr = T_w_last * T_last_curr;
  current_frame->setPose(T_w_curr);

  // ── Step 4: Propagate MapPoint associations from last_frame_ ──
  // If last_frame_ had a MapPoint at keypoint index m.queryIdx,
  // then current_frame inherits it at m.trainIdx.
  // This gives us tracked MapPoints for the KeyFrame decision.
  current_frame->ensureMapPointVectorSized(
      current_frame->getKeypoints().size());

  for (const auto &m : inliers_matches) {
    size_t idx_prev = m.queryIdx;
    size_t idx_curr = m.trainIdx;
    if (idx_prev < last_frame_->getMapPoints().size()) {
      MapPoint::Ptr mp = last_frame_->getMapPoints()[idx_prev];
      if (mp && !mp->isBad_) {
        current_frame->accessMapPoints()[idx_curr] = mp;
      }
    }
  }

  size_t tracked = countTrackedMapPoints(current_frame);

  // ── Step 5: KeyFrame decision ──
  if (shouldInsertKeyFrame(current_frame)) {
    insertKeyFrame(current_frame);
  }

  // ── Logging ──
  std::cout << "[Frontend] Inliers: " << inliers_count << "/" << matches.size()
            << " | Tracked MPs: " << tracked
            << " | Total KFs: " << map_->numKeyFrames()
            << " | Total MPs: " << map_->numMapPoints()
            << " | Pose: " << T_w_curr.translation().transpose() << std::endl;

  return true;
}

// ─────────────────────────────────────────────────────────────────
// KeyFrame insertion decision
// ─────────────────────────────────────────────────────────────────
bool VisualFrontend::shouldInsertKeyFrame(Frame::Ptr current_frame) const {
  // 1. Minimum temporal spacing
  if (frames_since_last_kf_ < kMinFramesBetweenKF)
    return false;

  // 2. Tracking quality: if too few MapPoints are tracked → need new KF
  size_t tracked = countTrackedMapPoints(current_frame);
  if ((int)tracked < kMinTrackedMapPoints)
    return true;

  // 3. Sufficient baseline (translation) from last KeyFrame
  gtsam::Pose3 T_kf_curr =
      last_keyframe_->getPose().inverse() * current_frame->getPose();
  double baseline = T_kf_curr.translation().norm();
  if (baseline > kMinBaseline)
    return true;

  // 4. Sufficient rotation from last KeyFrame
  // axisAngle() returns pair<Unit3, double> — second element is angle in
  // radians
  Eigen::AngleAxisd aa(T_kf_curr.rotation().matrix());
  double angle_deg = aa.angle() * 180.0 / M_PI;
  if (angle_deg > kMinRotationDeg)
    return true;

  return false;
}

// ─────────────────────────────────────────────────────────────────
// insertKeyFrame
// ─────────────────────────────────────────────────────────────────
void VisualFrontend::insertKeyFrame(Frame::Ptr current_frame) {
  KeyFrame::Ptr new_kf = KeyFrame::create(current_frame);
  map_->addKeyFrame(new_kf);

  // Triangulate new MapPoints between last_keyframe_ and new_kf
  triangulateNewPoints(last_keyframe_, new_kf);

  // Update covisibility for both
  new_kf->updateCovisibility();
  last_keyframe_->updateCovisibility();

  // ──────────────────────────────────────────────────────
  // CRITICAL: Sync MapPoints back into the Frame
  // so that the NEXT track() call can propagate them.
  // ──────────────────────────────────────────────────────
  new_kf->syncMapPointsToFrame(current_frame);

  // Advance KeyFrame pointers
  reference_keyframe_ = new_kf;
  last_keyframe_ = new_kf;
  frames_since_last_kf_ = 0;

  // Periodic cleanup
  map_->cleanBadMapPoints();

  std::cout << "[Frontend] ★ New KF " << new_kf->getId() << " (key "
            << gtsam::Symbol(new_kf->getPoseKey()) << ")"
            << " | KF MPs: " << new_kf->countMapPoints()
            << " | Map MPs: " << map_->numMapPoints() << std::endl;
}

// ─────────────────────────────────────────────────────────────────
// triangulateNewPoints between two KeyFrames
// ─────────────────────────────────────────────────────────────────
void VisualFrontend::triangulateNewPoints(KeyFrame::Ptr kf1,
                                          KeyFrame::Ptr kf2) {
  // Match descriptors between the two KeyFrames
  // We need a temporary Frame-like interface — use descriptors directly.
  // Reuse FeatureMatcher: create temporary frames wrapping KeyFrame data.
  // (Alternatively, add a match(KeyFrame, KeyFrame) overload later)

  // For now: build matched points from descriptor matching
  std::vector<std::vector<cv::DMatch>> knn_matches;
  auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->knnMatch(kf1->getDescriptors(), kf2->getDescriptors(), knn_matches,
                    2);

  // Ratio test
  std::vector<cv::DMatch> good_matches;
  for (const auto &knn : knn_matches) {
    if (knn.size() < 2)
      continue;
    if (knn[0].distance < 0.7f * knn[1].distance) {
      good_matches.push_back(knn[0]);
    }
  }

  if (good_matches.empty())
    return;

  // Filter: skip keypoints that already have a MapPoint in BOTH KeyFrames
  std::vector<cv::Point2f> pts1, pts2;
  std::vector<cv::DMatch> matches_to_triangulate;
  for (const auto &m : good_matches) {
    bool has_mp1 = (kf1->getMapPoints()[m.queryIdx] != nullptr);
    bool has_mp2 = (kf2->getMapPoints()[m.trainIdx] != nullptr);

    if (has_mp1 && has_mp2) {
      // Both already have MapPoints — check if it's the SAME one (merge
      // opportunity) For now, skip
      continue;
    }
    if (has_mp1 && !has_mp2) {
      // kf1 has a MapPoint, kf2 doesn't → just extend the observation
      MapPoint::Ptr mp = kf1->getMapPoints()[m.queryIdx];
      if (mp && !mp->isBad_) {
        mp->addObservation(kf2, m.trainIdx);
        kf2->accessMapPoints()[m.trainIdx] = mp;
      }
      continue;
    }
    if (!has_mp1 && has_mp2) {
      // Same, reverse direction
      MapPoint::Ptr mp = kf2->getMapPoints()[m.trainIdx];
      if (mp && !mp->isBad_) {
        mp->addObservation(kf1, m.queryIdx);
        kf1->accessMapPoints()[m.queryIdx] = mp;
      }
      continue;
    }

    // Neither has a MapPoint → triangulate
    pts1.push_back(kf1->getKeypoints()[m.queryIdx].pt);
    pts2.push_back(kf2->getKeypoints()[m.trainIdx].pt);
    matches_to_triangulate.push_back(m);
  }

  if (pts1.empty())
    return;

  // Compute relative pose: kf1 → kf2
  gtsam::Pose3 T_w_1 = kf1->getPose();
  gtsam::Pose3 T_w_2 = kf2->getPose();
  gtsam::Pose3 T_1_2 = T_w_1.inverse() * T_w_2;

  // Convert to OpenCV for triangulation
  Eigen::Matrix3d R_eigen = T_1_2.rotation().matrix();
  Eigen::Vector3d t_eigen = T_1_2.translation();

  cv::Mat R_cv(3, 3, CV_64F), t_cv(3, 1, CV_64F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      R_cv.at<double>(i, j) = R_eigen(i, j);
    t_cv.at<double>(i) = t_eigen(i);
  }

  // Triangulate
  std::vector<cv::Point3f> points_3d;
  pose_estimator_->triangulate(pts1, pts2, K_, R_cv, t_cv, points_3d);

  // Validate and create MapPoints
  const double max_range = 50.0;
  const double min_parallax_deg = 1.0;
  const double min_parallax_rad = min_parallax_deg * M_PI / 180.0;

  size_t created = 0;

  for (size_t i = 0; i < points_3d.size(); i++) {
    const cv::Point3f &p = points_3d[i];
    cv::Mat pt = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

    // Cheirality: positive depth in both cameras
    double z1 = pt.at<double>(2);
    if (z1 <= 0)
      continue;

    cv::Mat pt_2 = R_cv * pt + t_cv;
    double z2 = pt_2.at<double>(2);
    if (z2 <= 0)
      continue;

    // Range check
    double dist = cv::norm(pt);
    if (dist > max_range)
      continue;

    // Parallax check
    cv::Mat b1 = pt / cv::norm(pt);
    cv::Mat b2 = pt_2 / cv::norm(pt_2);
    double dot_val = b1.dot(b2);
    dot_val = std::max(-1.0, std::min(1.0, dot_val));
    double parallax = std::acos(dot_val);
    if (parallax < min_parallax_rad)
      continue;

    // Transform to world coordinates (from kf1's camera frame)
    gtsam::Point3 p_cam1(p.x, p.y, p.z);
    gtsam::Point3 p_world = T_w_1.transformFrom(p_cam1);

    Eigen::Vector3d pos_world(p_world.x(), p_world.y(), p_world.z());
    MapPoint::Ptr mp = MapPoint::create(pos_world);

    // Register observations
    const cv::DMatch &m = matches_to_triangulate[i];
    mp->addObservation(kf1, m.queryIdx);
    mp->addObservation(kf2, m.trainIdx);

    kf1->accessMapPoints()[m.queryIdx] = mp;
    kf2->accessMapPoints()[m.trainIdx] = mp;

    // Compute representative descriptor
    mp->computeDistinctiveDescriptor();

    // Add to global map
    map_->addMapPoint(mp);
    created++;
  }

  std::cout << "[Frontend] Triangulated " << created
            << " new MapPoints between KF " << kf1->getId() << " and KF "
            << kf2->getId() << std::endl;
}

// ─────────────────────────────────────────────────────────────────
// Utility
// ─────────────────────────────────────────────────────────────────
size_t VisualFrontend::countTrackedMapPoints(Frame::Ptr frame) const {
  size_t count = 0;
  for (const auto &mp : frame->getMapPoints()) {
    if (mp && !mp->isBad_)
      count++;
  }
  return count;
}

Frame::Ptr VisualFrontend::getLatestFrame() const {
    return last_frame_;
}

KeyFrame::Ptr VisualFrontend::getLatestKeyFrame() const {
  return last_keyframe_;
}

} // namespace frontend