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
  pose_estimator_->setIntrinsics(K);
  K_ = K.clone();
  map_ = std::make_shared<Map>();
}

bool VisualFrontend::handleImage(const cv::Mat &gray_image, double timestamp) {
  Frame::Ptr frame = Frame::createFrame(gray_image, timestamp);
  extractFeatures(frame);
  bool success = process(frame);

  if (viewer_) {
    cv::Mat img_out;
    cv::drawKeypoints(frame->getImage(), frame->getKeypoints(), img_out);
    viewer_->show(img_out);
  }
  return success;
}

void VisualFrontend::extractFeatures(Frame::Ptr frame) {
  feature_extractor_->extract(*frame);
}

std::optional<FrontendOutput> VisualFrontend::consumeBackendOutput() {
  auto output = pending_output_;
  pending_output_.reset();
  return output;
}

// ─────────────────────────────────────────────────────────────────
// STATE MACHINE
// ─────────────────────────────────────────────────────────────────
bool VisualFrontend::process(Frame::Ptr current_frame) {
  current_frame_ = current_frame;

  switch (stage_) {

  case Stage::NO_IMAGES_YET: {
    current_frame->setPose(gtsam::Pose3());
    last_keyframe_ = KeyFrame::create(current_frame);
    reference_keyframe_ = last_keyframe_;
    map_->addKeyFrame(last_keyframe_);
    last_frame_ = current_frame;
    frames_since_last_kf_ = 0;
    stage_ = Stage::INITIALIZING;

    // Emit prior for first KF
    FrontendOutput output;
    output.is_first_keyframe = true;
    output.current_key = last_keyframe_->getPoseKey();
    output.prior_pose = gtsam::Pose3();
    output.initial_estimate = gtsam::Pose3();
    output.timestamp = current_frame->getTimestamp();
    pending_output_ = output;

    std::cout << "[Frontend] First frame → KF " << last_keyframe_->getId()
              << std::endl;
    return false;
  }

  case Stage::INITIALIZING: {
    bool ok = tryInitialize(current_frame);
    if (ok) {
      stage_ = Stage::TRACKING;
      std::cout << "[Frontend] Initialization → TRACKING" << std::endl;
    }
    last_frame_ = current_frame;
    frames_since_last_kf_++;
    return ok;
  }

  case Stage::TRACKING: {
    bool ok = track(current_frame);
    if (ok) {
      if (last_frame_) {
        velocity_ = last_frame_->getPose().inverse() * current_frame->getPose();
        has_velocity_ = true;
      }
      last_frame_ = current_frame;
      consecutive_failures_ = 0;
    } else {
      consecutive_failures_++;
      // Keep features current but hold last good pose
      current_frame->setPose(last_frame_->getPose());
      last_frame_ = current_frame;
      if (consecutive_failures_ > kMaxConsecutiveFailures) {
        has_velocity_ = false;
      }
    }
    frames_since_last_kf_++;
    return ok;
  }

  } // end switch
  return false;
}

// ─────────────────────────────────────────────────────────────────
// INITIALIZATION (Essential matrix — only used here, no map yet)
// ─────────────────────────────────────────────────────────────────
bool VisualFrontend::tryInitialize(Frame::Ptr current_frame) {
  std::vector<cv::DMatch> matches =
      feature_matcher_->match(last_frame_, current_frame);
  if (matches.size() < 20)
    return false;

  std::vector<cv::Point2f> pts_prev, pts_curr;
  for (const auto &m : matches) {
    pts_prev.push_back(last_frame_->getKeypoints()[m.queryIdx].pt);
    pts_curr.push_back(current_frame->getKeypoints()[m.trainIdx].pt);
  }

  cv::Mat R, t, mask;
  if (!pose_estimator_->estimate(pts_prev, pts_curr, K_, R, t, mask))
    return false;

  int inlier_count = cv::countNonZero(mask);
  if (inlier_count < 20)
    return false;

  gtsam::Pose3 T_last_curr = cvToGtsam(R, t);
  double baseline = T_last_curr.translation().norm();
  if (baseline < kMinBaseline)
    return false;

  gtsam::Pose3 T_w_prev = last_keyframe_->getPose();
  gtsam::Pose3 T_w_curr = T_w_prev * T_last_curr;
  current_frame->setPose(T_w_curr);

  KeyFrame::Ptr curr_kf = KeyFrame::create(current_frame);
  map_->addKeyFrame(curr_kf);
  triangulateNewPoints(last_keyframe_, curr_kf);
  last_keyframe_->updateCovisibility();
  curr_kf->updateCovisibility();
  curr_kf->syncMapPointsToFrame(current_frame);

  // Emit BetweenFactor for backend
  FrontendOutput output;
  output.is_first_keyframe = false;
  output.previous_key = last_keyframe_->getPoseKey();
  output.current_key = curr_kf->getPoseKey();
  output.relative_pose = T_last_curr;
  output.initial_estimate = T_w_curr;
  output.timestamp = current_frame->getTimestamp();
  pending_output_ = output;

  last_keyframe_ = curr_kf;
  reference_keyframe_ = curr_kf;
  frames_since_last_kf_ = 0;

  std::cout << "[Frontend Init] KF " << curr_kf->getId()
            << " | baseline=" << baseline << " | MPs=" << map_->numMapPoints()
            << std::endl;
  return true;
}

// ─────────────────────────────────────────────────────────────────
// TRACKING: projection + PnP only, NO Essential fallback
// ─────────────��───────────────────────────────────────────────────
bool VisualFrontend::track(Frame::Ptr current_frame) {
  predictPose(current_frame);

  bool ok = trackWithLocalMap(current_frame);
  if (!ok)
    return false;

  if (shouldInsertKeyFrame(current_frame)) {
    insertKeyFrame(current_frame);
  }
  return true;
}

void VisualFrontend::predictPose(Frame::Ptr current_frame) {
  if (has_velocity_ && last_frame_) {
    current_frame->setPose(last_frame_->getPose() * velocity_);
  } else if (last_frame_) {
    current_frame->setPose(last_frame_->getPose());
  }
}

bool VisualFrontend::trackWithLocalMap(Frame::Ptr current_frame) {
  current_frame->ensureMapPointVectorSized(
      current_frame->getKeypoints().size());

  // Phase 1: Propagate MapPoints from last_frame_
  size_t propagated = 0;
  if (last_frame_) {
    auto matches = feature_matcher_->match(last_frame_, current_frame);
    for (const auto &m : matches) {
      size_t idx_prev = m.queryIdx;
      size_t idx_curr = m.trainIdx;
      if (idx_prev < last_frame_->getMapPoints().size()) {
        MapPoint::Ptr mp = last_frame_->getMapPoints()[idx_prev];
        if (mp && !mp->isBad_) {
          current_frame->accessMapPoints()[idx_curr] = mp;
          propagated++;
        }
      }
    }
  }

  // Phase 2: Project local map
  auto local_mps = map_->getLocalMapPoints(reference_keyframe_, 10);
  int n_proj = feature_matcher_->matchByProjection(
      current_frame, local_mps, K_, current_frame->getPose(), kSearchRadius);
  size_t total_tracked = countTrackedMapPoints(current_frame);

  if ((int)total_tracked < kMinProjectionMatches) {
    n_proj += feature_matcher_->matchByProjection(current_frame, local_mps, K_,
                                                  current_frame->getPose(),
                                                  kSearchRadiusWide);
    total_tracked = countTrackedMapPoints(current_frame);
  }

  // Phase 3: Relocalization against reference KF
  if ((int)total_tracked < kMinProjectionMatches && reference_keyframe_) {
    std::vector<std::vector<cv::DMatch>> knn_matches;
    auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    if (!reference_keyframe_->getDescriptors().empty() &&
        !current_frame->getDescriptors().empty()) {
      matcher->knnMatch(reference_keyframe_->getDescriptors(),
                        current_frame->getDescriptors(), knn_matches, 2);
      for (const auto &knn : knn_matches) {
        if (knn.size() < 2)
          continue;
        if (knn[0].distance < 0.7f * knn[1].distance) {
          size_t kf_idx = knn[0].queryIdx;
          size_t fr_idx = knn[0].trainIdx;
          if (current_frame->accessMapPoints()[fr_idx])
            continue;
          if (kf_idx < reference_keyframe_->getMapPoints().size()) {
            MapPoint::Ptr mp = reference_keyframe_->getMapPoints()[kf_idx];
            if (mp && !mp->isBad_) {
              current_frame->accessMapPoints()[fr_idx] = mp;
            }
          }
        }
      }
      total_tracked = countTrackedMapPoints(current_frame);
    }
  }

  if ((int)total_tracked < 10) {
    std::cout << "[Frontend] Not enough MPs: propagated=" << propagated
              << " proj=" << n_proj << " total=" << total_tracked << std::endl;
    return false;
  }

  // Phase 4: PnP
  int pnp_inliers = 0;
  if (!pose_estimator_->estimateRefined(current_frame, &pnp_inliers)) {
    std::cout << "[Frontend] PnP failed (inliers=" << pnp_inliers << ")"
              << std::endl;
    return false;
  }

  // Phase 5: Post-PnP re-project with refined pose
  size_t after_pnp = countTrackedMapPoints(current_frame);
  if ((int)after_pnp < kMinTrackedMapPoints) {
    int extra = feature_matcher_->matchByProjection(
        current_frame, local_mps, K_, current_frame->getPose(), kSearchRadius);
    if (extra > 0) {
      pose_estimator_->estimateRefined(current_frame, &pnp_inliers);
    }
  }

  std::cout << "[Frontend] PnP OK | inliers=" << pnp_inliers
            << " | prop=" << propagated << " | proj=" << n_proj
            << " | final=" << countTrackedMapPoints(current_frame)
            << " | Pose: " << current_frame->getPose().translation().transpose()
            << std::endl;

  return true;
}

bool VisualFrontend::shouldInsertKeyFrame(Frame::Ptr current_frame) const {
  if (frames_since_last_kf_ < kMinFramesBetweenKF)
    return false;

  size_t tracked = countTrackedMapPoints(current_frame);
  if ((int)tracked < kMinTrackedMapPoints)
    return true;

  gtsam::Pose3 T_kf_curr =
      last_keyframe_->getPose().inverse() * current_frame->getPose();
  double baseline = T_kf_curr.translation().norm();
  if (baseline > kMinBaseline)
    return true;

  Eigen::AngleAxisd aa(T_kf_curr.rotation().matrix());
  if (aa.angle() * 180.0 / M_PI > kMinRotationDeg)
    return true;

  return false;
}

void VisualFrontend::insertKeyFrame(Frame::Ptr current_frame) {
  KeyFrame::Ptr new_kf = KeyFrame::create(current_frame);
  map_->addKeyFrame(new_kf);
  triangulateNewPoints(last_keyframe_, new_kf);
  new_kf->updateCovisibility();
  last_keyframe_->updateCovisibility();
  new_kf->syncMapPointsToFrame(current_frame);

  // ──────────────────────────────────────────────────────
  // EMIT BETWEEN FACTOR FOR GTSAM BACKEND
  // This is the key output that feeds the fixed-lag smoother.
  // ──────────────────────────────────────────────────────
  gtsam::Pose3 relative_pose =
      last_keyframe_->getPose().inverse() * new_kf->getPose();

  FrontendOutput output;
  output.is_first_keyframe = false;
  output.previous_key = last_keyframe_->getPoseKey();
  output.current_key = new_kf->getPoseKey();
  output.relative_pose = relative_pose;
  output.initial_estimate = new_kf->getPose();
  output.timestamp = current_frame->getTimestamp();
  pending_output_ = output;

  reference_keyframe_ = new_kf;
  last_keyframe_ = new_kf;
  frames_since_last_kf_ = 0;
  map_->cleanBadMapPoints();

  std::cout << "[Frontend] ★ New KF " << new_kf->getId()
            << " | KF MPs: " << new_kf->countMapPoints()
            << " | Map MPs: " << map_->numMapPoints() << std::endl;
}

void VisualFrontend::triangulateNewPoints(KeyFrame::Ptr kf1,
                                          KeyFrame::Ptr kf2) {
  std::vector<std::vector<cv::DMatch>> knn_matches;
  auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  matcher->knnMatch(kf1->getDescriptors(), kf2->getDescriptors(), knn_matches,
                    2);

  std::vector<cv::DMatch> good_matches;
  for (const auto &knn : knn_matches) {
    if (knn.size() < 2)
      continue;
    if (knn[0].distance < 0.7f * knn[1].distance)
      good_matches.push_back(knn[0]);
  }
  if (good_matches.empty())
    return;

  std::vector<cv::Point2f> pts1, pts2;
  std::vector<cv::DMatch> matches_to_triangulate;
  for (const auto &m : good_matches) {
    bool has1 = (kf1->getMapPoints()[m.queryIdx] != nullptr);
    bool has2 = (kf2->getMapPoints()[m.trainIdx] != nullptr);

    if (has1 && has2)
      continue;
    if (has1 && !has2) {
      MapPoint::Ptr mp = kf1->getMapPoints()[m.queryIdx];
      if (mp && !mp->isBad_) {
        mp->addObservation(kf2, m.trainIdx);
        kf2->accessMapPoints()[m.trainIdx] = mp;
      }
      continue;
    }
    if (!has1 && has2) {
      MapPoint::Ptr mp = kf2->getMapPoints()[m.trainIdx];
      if (mp && !mp->isBad_) {
        mp->addObservation(kf1, m.queryIdx);
        kf1->accessMapPoints()[m.queryIdx] = mp;
      }
      continue;
    }
    pts1.push_back(kf1->getKeypoints()[m.queryIdx].pt);
    pts2.push_back(kf2->getKeypoints()[m.trainIdx].pt);
    matches_to_triangulate.push_back(m);
  }
  if (pts1.empty())
    return;

  gtsam::Pose3 T_w_1 = kf1->getPose();
  gtsam::Pose3 T_1_2 = T_w_1.inverse() * kf2->getPose();

  Eigen::Matrix3d R_eigen = T_1_2.rotation().matrix();
  Eigen::Vector3d t_eigen = T_1_2.translation();
  cv::Mat R_cv(3, 3, CV_64F), t_cv(3, 1, CV_64F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      R_cv.at<double>(i, j) = R_eigen(i, j);
    t_cv.at<double>(i) = t_eigen(i);
  }

  std::vector<cv::Point3f> points_3d;
  pose_estimator_->triangulate(pts1, pts2, K_, R_cv, t_cv, points_3d);

  size_t created = 0;
  for (size_t i = 0; i < points_3d.size(); i++) {
    const cv::Point3f &p = points_3d[i];
    cv::Mat pt = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

    if (pt.at<double>(2) <= 0)
      continue;
    cv::Mat pt_2 = R_cv * pt + t_cv;
    if (pt_2.at<double>(2) <= 0)
      continue;
    if (cv::norm(pt) > 50.0)
      continue;

    cv::Mat b1 = pt / cv::norm(pt);
    cv::Mat b2 = pt_2 / cv::norm(pt_2);
    double dot_val = std::max(-1.0, std::min(1.0, b1.dot(b2)));
    if (std::acos(dot_val) < 1.0 * M_PI / 180.0)
      continue;

    gtsam::Point3 p_world = T_w_1.transformFrom(gtsam::Point3(p.x, p.y, p.z));
    MapPoint::Ptr mp = MapPoint::create(
        Eigen::Vector3d(p_world.x(), p_world.y(), p_world.z()));

    const cv::DMatch &m = matches_to_triangulate[i];
    mp->addObservation(kf1, m.queryIdx);
    mp->addObservation(kf2, m.trainIdx);
    kf1->accessMapPoints()[m.queryIdx] = mp;
    kf2->accessMapPoints()[m.trainIdx] = mp;
    mp->computeDistinctiveDescriptor();
    map_->addMapPoint(mp);
    created++;
  }

  std::cout << "[Frontend] Triangulated " << created
            << " new MapPoints between KF " << kf1->getId() << " and KF "
            << kf2->getId() << std::endl;
}

size_t VisualFrontend::countTrackedMapPoints(Frame::Ptr frame) const {
  size_t count = 0;
  for (const auto &mp : frame->getMapPoints()) {
    if (mp && !mp->isBad_)
      count++;
  }
  return count;
}

Frame::Ptr VisualFrontend::getLatestFrame() const { return last_frame_; }
KeyFrame::Ptr VisualFrontend::getLatestKeyFrame() const {
  return last_keyframe_;
}

} // namespace frontend