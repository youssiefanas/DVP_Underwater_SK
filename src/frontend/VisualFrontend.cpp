#include "frontend/VisualFrontend.hpp"
#include "dv_slam/utility.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace frontend {

// ─────────────────────────────────────────────────────────────────
// STATIC HELPERS
// ─────────────────────────────────────────────────────────────────

std::vector<cv::DMatch> VisualFrontend::extractValidMatchedPoints(
    const std::vector<cv::DMatch> &matches,
    const std::vector<cv::KeyPoint> &kps1,
    const std::vector<cv::KeyPoint> &kps2, std::vector<cv::Point2f> &pts1,
    std::vector<cv::Point2f> &pts2) {
  std::vector<cv::DMatch> valid;
  valid.reserve(matches.size());
  pts1.reserve(matches.size());
  pts2.reserve(matches.size());
  for (const auto &m : matches) {
    if (m.queryIdx < 0 || m.trainIdx < 0)
      continue;
    if (static_cast<size_t>(m.queryIdx) >= kps1.size() ||
        static_cast<size_t>(m.trainIdx) >= kps2.size())
      continue;
    pts1.push_back(kps1[m.queryIdx].pt);
    pts2.push_back(kps2[m.trainIdx].pt);
    valid.push_back(m);
  }
  return valid;
}

bool VisualFrontend::isTriangulatedPointValid(
    const cv::Point3f &p, const cv::Mat &R_cv, const cv::Mat &t_cv,
    const Eigen::Vector3d &cam1_world, const Eigen::Vector3d &cam2_world,
    const Pose3d &T_w_ref, double max_distance, double min_parallax_deg) {
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    return false;

  // Positive depth in camera 1
  if (p.z <= 0)
    return false;

  // Positive depth in camera 2
  cv::Mat pt = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);
  cv::Mat pt_2 = R_cv * pt + t_cv;
  if (pt_2.at<double>(2) <= 0)
    return false;

  // Distance bound
  if (cv::norm(pt) > max_distance)
    return false;

  // Parallax angle check
  Eigen::Vector3d p_world = T_w_ref * Eigen::Vector3d(p.x, p.y, p.z);
  Eigen::Vector3d ray1 = p_world - cam1_world;
  Eigen::Vector3d ray2 = p_world - cam2_world;
  double cos_par = ray1.dot(ray2) / (ray1.norm() * ray2.norm());
  cos_par = std::clamp(cos_par, -1.0, 1.0);
  if (std::acos(cos_par) < min_parallax_deg * M_PI / 180.0)
    return false;

  return true;
}

// ─────────────────────────────────────────────────────────────────
// CONSTRUCTION
// ─────────────────────────────────────────────────────────────────

VisualFrontend::VisualFrontend(const ORBParams &params, const cv::Mat &K,
                               const cv::Mat &dist_coeffs, double init_scale,
                               bool enable_viewer)
    : stage_(Stage::NO_IMAGES_YET), init_scale_(init_scale) {
  if (init_scale_ <= 0.0) {
    std::cerr << "[Frontend] Invalid init_scale (" << init_scale_
              << "), falling back to 1.0" << std::endl;
    init_scale_ = 1.0;
  }
  feature_extractor_ = std::make_shared<FeatureExtractor>(params);
  if (enable_viewer) {
    viewer_ = std::make_shared<Viewer>();
  }
  feature_matcher_ = std::make_shared<FeatureMatcher>(params.matcher_type);
  pose_estimator_ = std::make_shared<PoseEstimator>();
  pose_estimator_->setIntrinsics(K, dist_coeffs);
  K_ = K.clone();
  map_ = std::make_shared<Map>();
}

// ─────────────────────────────────────────────────────────────────
// PUBLIC ENTRY POINT
// ─────────────────────────────────────────────────────────────────

bool VisualFrontend::handleImage(const cv::Mat &gray_image, double timestamp) {
  using Clock = std::chrono::steady_clock;
  if (gray_image.empty()) {
    std::cerr << "[Frontend] Received empty image, skipping frame."
              << std::endl;
    return false;
  }

  const auto t0 = Clock::now();

  Frame::Ptr frame = Frame::createFrame(gray_image, timestamp);
  extractFeatures(frame);

  const auto t1 = Clock::now();

  if (frame->getKeypoints().empty() || frame->getDescriptors().empty()) {
    std::cout << "[Frontend] No features extracted on frame " << frame->getId()
              << ", skipping." << std::endl;
    return false;
  }

  bool success = process(frame);

  const auto t2 = Clock::now();

  if (viewer_) {
    cv::Mat img_out;
    cv::drawKeypoints(frame->getImage(), frame->getKeypoints(), img_out);
    viewer_->show(img_out);
  }

  const auto t3 = Clock::now();

  auto ms = [](auto a, auto b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  last_timing_.extraction_ms = ms(t0, t1);
  last_timing_.tracking_ms   = ms(t1, t2);
  last_timing_.viewer_ms     = ms(t2, t3);
  last_timing_.total_ms      = ms(t0, t3);

  return success;
}

void VisualFrontend::extractFeatures(Frame::Ptr frame) {
  feature_extractor_->extract(*frame);
}

// ─────────────────────────────────────────────────────────────────
// STATE MACHINE
// ─────────────────────────────────────────────────────────────────

bool VisualFrontend::process(Frame::Ptr current_frame) {
  current_frame_ = current_frame;

  switch (stage_) {

  case Stage::NO_IMAGES_YET: {
    if (initial_pose_override_) {
      last_good_pose_ = *initial_pose_override_;
      initial_pose_override_.reset();
    }
    current_frame->setPose(last_good_pose_);
    last_keyframe_ = KeyFrame::create(current_frame);
    last_keyframe_->registerMapPointObservations();
    reference_keyframe_ = last_keyframe_;
    map_->addKeyFrame(last_keyframe_);
    last_frame_ = current_frame;
    frames_since_last_kf_ = 0;
    init_attempts_ = 0;
    stage_ = Stage::INITIALIZING;

    FrontendOutput output;
    output.is_first_keyframe = true;
    output.prior_pose = last_good_pose_;
    output.initial_estimate = last_good_pose_;
    output.timestamp = current_frame->getTimestamp();
    // Tight prior: first KF defines the origin
    output.pose_covariance = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    pending_output_ = output;

    std::cout << "[Frontend] First frame → KF " << last_keyframe_->getId()
              << std::endl;
    return false;
  }

  case Stage::INITIALIZING: {
    bool ok = tryInitialize(current_frame);
    if (ok) {
      stage_ = Stage::TRACKING;
      last_frame_ = current_frame;
      init_attempts_ = 0;
      std::cout << "[Frontend] Initialization → TRACKING" << std::endl;
    } else {
      init_attempts_++;
      if (init_attempts_ >= kMaxInitAttempts) {
        std::cout << "[Frontend Init] Stuck after " << init_attempts_
                  << " attempts → refreshing reference KF" << std::endl;
        refreshInitializationReference(current_frame);
        init_attempts_ = 0;
      }
    }
    frames_since_last_kf_++;
    return ok;
  }

  case Stage::TRACKING: {
    bool ok = track(current_frame);
    // TODO: if tracking fails, we could still publish the last good pose with
    // increased covariance instead of skipping the frame entirely
    // TODO: get last pose from mimosa instead of dead reckoning with the last
    // velocity (which can be very wrong if we lose tracking for a few frames or
    // the corrected trajectrory is different as mimosa is optimized with other
    // sensors)
    if (ok) {
      if (last_frame_) {
        velocity_ =
            last_frame_->getPose().inverse() * current_frame->getPose();
        has_velocity_ = true;
      }
      last_good_pose_ = current_frame->getPose();
      last_frame_ = current_frame;
      consecutive_failures_ = 0;
    } else {
      consecutive_failures_++;
      if (last_frame_) {
        current_frame->setPose(last_frame_->getPose());
      }
      if (consecutive_failures_ >= kMaxConsecutiveFailures) {
        std::cout << "[Frontend] Tracking lost after " << consecutive_failures_
                  << " consecutive failures → LOST" << std::endl;
        stage_ = Stage::LOST;
        has_velocity_ = false;
        lost_frames_ = 0;
      }
    }
    frames_since_last_kf_++;
    return ok;
  }

  case Stage::LOST: {
    lost_frames_++;
    bool ok = tryRelocalize(current_frame);
    if (ok) {
      std::cout << "[Frontend] Relocalized → TRACKING" << std::endl;
      stage_ = Stage::TRACKING;
      consecutive_failures_ = 0;
      lost_frames_ = 0;
      if (last_frame_) {
        velocity_ = last_frame_->getPose().inverse() * current_frame->getPose();
        has_velocity_ = true;
      }
      last_good_pose_ = current_frame->getPose();
      last_frame_ = current_frame;
      return true;
    }
    if (lost_frames_ >= kMaxRelocFrames) {
      std::cout << "[Frontend] Relocalization failed for " << lost_frames_
                << " frames → re-initializing" << std::endl;
      resetToInitializing();
    }
    return false;
  }

  } // end switch
  return false;
}

// ─────────────────────────────────────────────────────────────────
// INITIALIZATION (Essential matrix)
// ─────────────────────────────────────────────────────────────────

bool VisualFrontend::tryInitialize(Frame::Ptr current_frame) {
  // Log first few attempts at full rate, then every 10th to avoid spam.
  const bool log_attempt =
      (init_attempts_ < 3) || (init_attempts_ % 10 == 0);

  if (!last_frame_ || !last_keyframe_) {
    if (log_attempt)
      std::cout << "[Frontend Init] missing reference frame/KF" << std::endl;
    return false;
  }

  // Match features between first keyframe and current frame
  auto matches = feature_matcher_->match(last_keyframe_->getDescriptors(),
                                         current_frame->getDescriptors(),
                                         kMatchRatioThreshold);
  if (matches.size() < kMinMatchesForInit) {
    if (log_attempt)
      std::cout << "[Frontend Init] too few descriptor matches: "
                << matches.size() << " (need ≥ " << kMinMatchesForInit << ")"
                << " [attempt " << init_attempts_ << "]" << std::endl;
    return false;
  }

  std::vector<cv::Point2f> pts_prev, pts_curr;
  auto valid_matches = extractValidMatchedPoints(
      matches, last_keyframe_->getKeypoints(), current_frame->getKeypoints(),
      pts_prev, pts_curr);
  if (pts_prev.size() < kMinMatchesForInit) {
    if (log_attempt)
      std::cout << "[Frontend Init] too few valid matches after filtering: "
                << pts_prev.size() << " [attempt " << init_attempts_ << "]"
                << std::endl;
    return false;
  }

  // Essential matrix estimation
  cv::Mat R, t, mask;
  if (!pose_estimator_->estimate(pts_prev, pts_curr, K_, R, t, mask)) {
    if (log_attempt)
      std::cout << "[Frontend Init] essential matrix estimation failed"
                << " (matches=" << pts_prev.size() << ")"
                << " [attempt " << init_attempts_ << "]" << std::endl;
    return false;
  }

  int inlier_count = cv::countNonZero(mask);
  if (static_cast<size_t>(inlier_count) < kMinMatchesForInit) {
    if (log_attempt)
      std::cout << "[Frontend Init] too few essential-matrix inliers: "
                << inlier_count << " / " << pts_prev.size()
                << " [attempt " << init_attempts_ << "]" << std::endl;
    return false;
  }

  // recoverPose returns T_curr_prev (x_curr = R*x_prev + t). We store T_w_c.
  Pose3d T_prev_curr = cvToPose3d(R, t).inverse();
  const double baseline_raw = T_prev_curr.translation().norm();
  if (baseline_raw < kMinBaseline) {
    if (log_attempt)
      std::cout << "[Frontend Init] insufficient baseline: " << baseline_raw
                << " (need ≥ " << kMinBaseline << ", likely near-pure rotation)"
                << " [attempt " << init_attempts_ << "]" << std::endl;
    return false;
  }

  // Filter matches and points by geometric inlier mask
  std::vector<cv::DMatch> inlier_matches;
  std::vector<cv::Point2f> pts_prev_inlier, pts_curr_inlier;
  inlier_matches.reserve(inlier_count);
  pts_prev_inlier.reserve(inlier_count);
  pts_curr_inlier.reserve(inlier_count);
  for (int i = 0; i < mask.rows; i++) {
    if (mask.at<uchar>(i)) {
      inlier_matches.push_back(valid_matches[i]);
      pts_prev_inlier.push_back(pts_prev[i]);
      pts_curr_inlier.push_back(pts_curr[i]);
    }
  }

  // Apply monocular scale
  if (std::abs(init_scale_ - 1.0) > 1e-12) {
    Eigen::Vector3d t_scaled = T_prev_curr.translation() * init_scale_;
    T_prev_curr = makePose(T_prev_curr.linear(), t_scaled);
  }
  double baseline = T_prev_curr.translation().norm();

  Pose3d T_w_prev = last_keyframe_->getPose();
  Pose3d T_w_curr = T_w_prev * T_prev_curr;
  current_frame->setPose(T_w_curr);

  KeyFrame::Ptr curr_kf = KeyFrame::create(current_frame);
  curr_kf->registerMapPointObservations();
  map_->addKeyFrame(curr_kf);

  // Triangulate initial map points using scaled relative pose
  Pose3d T_curr_prev_scaled = T_w_curr.inverse() * T_w_prev;
  cv::Mat R_d, t_d;
  pose3dToCv(T_curr_prev_scaled, R_d, t_d);

  std::vector<cv::Point3f> points_3d;
  pose_estimator_->triangulate(pts_prev_inlier, pts_curr_inlier, K_, R_d, t_d,
                               points_3d);

  Eigen::Vector3d cam1_w = T_w_prev.translation();
  Eigen::Vector3d cam2_w = T_w_curr.translation();

  size_t created = 0;
  for (size_t i = 0; i < points_3d.size() && i < inlier_matches.size(); i++) {
    if (!isTriangulatedPointValid(points_3d[i], R_d, t_d, cam1_w, cam2_w,
                                  T_w_prev, kMaxTriangulationDist,
                                  kMinParallaxDeg))
      continue;

    const auto &p = points_3d[i];
    Eigen::Vector3d P_w = T_w_prev * Eigen::Vector3d(p.x, p.y, p.z);
    MapPoint::Ptr mp = MapPoint::create(P_w);
    const auto &m = inlier_matches[i];
    mp->addObservation(last_keyframe_, m.queryIdx);
    mp->addObservation(curr_kf, m.trainIdx);
    last_keyframe_->accessMapPoints()[m.queryIdx] = mp;
    curr_kf->accessMapPoints()[m.trainIdx] = mp;
    mp->computeDistinctiveDescriptor();
    map_->addMapPoint(mp);
    created++;
  }

  if (static_cast<int>(created) < kMinInitMapPoints) {
    rollbackFailedInitialization(curr_kf);
    std::cout << "[Frontend Init] Reject initialization: triangulated MPs="
              << created << " (< " << kMinInitMapPoints << ")" << std::endl;
    return false;
  }

  last_keyframe_->updateCovisibility();
  curr_kf->updateCovisibility();
  curr_kf->syncMapPointsToFrame(current_frame);

  emitBackendOutput(false, T_prev_curr, T_w_curr,
                    current_frame->getTimestamp());

  last_keyframe_ = curr_kf;
  reference_keyframe_ = curr_kf;
  frames_since_last_kf_ = 0;

  std::cout << "[Frontend Init] KF " << curr_kf->getId()
            << " | baseline=" << baseline << " (raw=" << baseline_raw << ")"
            << " | MPs=" << map_->numMapPoints() << std::endl;
  return true;
}

// ─────────────────────────────────────────────────────────────────
// TRACKING
// ─────────────────────────────────────────────────────────────────

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
  // Highest priority: a fresh MIMOSA pose hint extrapolated to this image's
  // timestamp. Single-shot — consumed and cleared each call.
  if (pending_hint_ && pending_hint_->valid &&
      std::abs(current_frame->getTimestamp() - pending_hint_->timestamp) <=
          max_hint_age_s_) {
    current_frame->setPose(pending_hint_->T_w_cam);
    pending_hint_.reset();
    return;
  }
  pending_hint_.reset();  // stale → drop

  if (has_velocity_ && last_frame_) {
    current_frame->setPose(last_frame_->getPose() * velocity_);
  } else if (last_frame_) {
    current_frame->setPose(last_frame_->getPose());
  }
}

bool VisualFrontend::trackWithLocalMap(Frame::Ptr current_frame) {
  if (!current_frame)
    return false;

  const Pose3d predicted_pose = current_frame->getPose();
  current_frame->ensureMapPointVectorSized(
      current_frame->getKeypoints().size());

  // Phase 1: Propagate MapPoints from last_frame_
  size_t propagated = propagateMapPoints(current_frame);

  // Phase 2: Project local map (narrow, then wide fallback)
  auto local_mps = map_->getLocalMapPoints(reference_keyframe_, 10);
  int n_proj = feature_matcher_->matchByProjection(
      current_frame, local_mps, K_, current_frame->getPose(), kSearchRadius);
  size_t total_tracked = countTrackedMapPoints(current_frame);

  if (static_cast<int>(total_tracked) < kMinProjectionMatches) {
    n_proj += feature_matcher_->matchByProjection(current_frame, local_mps, K_,
                                                  current_frame->getPose(),
                                                  kSearchRadiusWide);
    total_tracked = countTrackedMapPoints(current_frame);
  }

  // Phase 3: Relocalization against reference KF
  if (static_cast<int>(total_tracked) < kMinProjectionMatches &&
      reference_keyframe_) {
    relocateFromReferenceKF(current_frame);
    total_tracked = countTrackedMapPoints(current_frame);
  }

  if (static_cast<int>(total_tracked) < kMinPnPInliers) {
    std::cout << "[Frontend] Not enough MPs: propagated=" << propagated
              << " proj=" << n_proj << " total=" << total_tracked << std::endl;
    return false;
  }

  // Phase 4: PnP RANSAC (initial pose + outlier rejection)
  int pnp_inliers = 0;
  if (!solvePnP(current_frame, pnp_inliers))
    return false;

  // Phase 5: Motion-only Bundle Adjustment (refines pose, computes covariance)
  // Optionally include a DVL translation prior to inject metric scale.
  PoseEstimator::DvlTranslationPrior dvl_prior;
  if (last_frame_) {
    Eigen::Vector3d dp_w;
    Eigen::Matrix3d cov_w;
    if (buildDvlPrior(last_frame_->getTimestamp(),
                      current_frame->getTimestamp(), last_frame_->getPose(),
                      &dp_w, &cov_w)) {
      dvl_prior.T_w_prev = last_frame_->getPose();
      dvl_prior.dp_world = dp_w;
      dvl_prior.cov_world = cov_w;
      dvl_prior.valid = true;
    }
  }

  Eigen::Matrix<double, 6, 6> covariance;
  int ba_inliers = 0;
  if (!pose_estimator_->motionOnlyBA(current_frame, &ba_inliers, &covariance,
                                     dvl_prior.valid ? &dvl_prior : nullptr)) {
    std::cout << "[Frontend] Motion-only BA failed" << std::endl;
    return false;
  }
  last_covariance_ = covariance;

  // Scale-lock accounting: accumulate VO and DVL travel since init, then
  // fire a one-shot Sim(3) rescale once we've moved enough.
  if (!scale_locked_ && last_frame_) {
    const Eigen::Vector3d dp_vo = current_frame->getPose().translation() -
                                  last_frame_->getPose().translation();
    cum_vo_dist_ += dp_vo.norm();
    if (dvl_prior.valid) {
      cum_dvl_dist_ += dvl_prior.dp_world.norm();
    }
    if (cum_vo_dist_ >= scale_lock_threshold_m_ && cum_dvl_dist_ > 0.0) {
      lockScale();
    }
  }

  // Phase 6: Reject implausible pose jumps
  if (!validatePoseJump(predicted_pose, current_frame->getPose(), ba_inliers))
    return false;

  std::cout << "[Frontend] BA OK | inliers=" << ba_inliers
            << " (PnP=" << pnp_inliers << ")"
            << " | prop=" << propagated << " | proj=" << n_proj
            << " | final=" << countTrackedMapPoints(current_frame)
            << " | Pose: " << current_frame->getPose().translation().transpose()
            << std::endl;

  return true;
}

// ─────────────────────────────────────────────────────────────────
// TRACKING SUB-STEPS
// ─────────────────────────────────────────────────────────────────

size_t VisualFrontend::propagateMapPoints(Frame::Ptr current_frame) {
  if (!last_frame_)
    return 0;

  size_t propagated = 0;
  auto matches = feature_matcher_->match(last_frame_, current_frame);
  for (const auto &m : matches) {
    if (m.queryIdx < 0 || m.trainIdx < 0)
      continue;
    size_t idx_prev = m.queryIdx;
    size_t idx_curr = m.trainIdx;
    if (idx_curr >= current_frame->accessMapPoints().size())
      continue;
    if (idx_prev < last_frame_->getMapPoints().size()) {
      MapPoint::Ptr mp = last_frame_->getMapPoints()[idx_prev];
      if (mp && !mp->isBad_) {
        current_frame->accessMapPoints()[idx_curr] = mp;
        propagated++;
      }
    }
  }
  return propagated;
}

void VisualFrontend::relocateFromReferenceKF(Frame::Ptr current_frame) {
  auto reloc_matches = feature_matcher_->match(
      reference_keyframe_->getDescriptors(), current_frame->getDescriptors(),
      kMatchRatioThreshold);
  for (const auto &m : reloc_matches) {
    size_t kf_idx = m.queryIdx;
    size_t fr_idx = m.trainIdx;
    if (fr_idx >= current_frame->accessMapPoints().size())
      continue;
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

bool VisualFrontend::solvePnP(Frame::Ptr current_frame, int &pnp_inliers) {
  if (!pose_estimator_->estimateRefined(current_frame, &pnp_inliers)) {
    std::cout << "[Frontend] PnP failed (inliers=" << pnp_inliers << ")"
              << std::endl;
    return false;
  }
  if (pnp_inliers < kMinPnPInliers) {
    std::cout << "[Frontend] Reject weak PnP (inliers=" << pnp_inliers << ")"
              << std::endl;
    return false;
  }
  return true;
}

bool VisualFrontend::validatePoseJump(const Pose3d &predicted_pose,
                                      const Pose3d &actual_pose,
                                      int pnp_inliers) const {
  Pose3d T_pred_curr = predicted_pose.inverse() * actual_pose;
  double pose_jump_t = T_pred_curr.translation().norm();
  double pose_jump_rot_deg =
      Eigen::AngleAxisd(T_pred_curr.linear()).angle() * 180.0 / M_PI;

  double max_jump_t = kMaxPoseJumpTranslation;
  if (has_velocity_) {
    double predicted_step_t = velocity_.translation().norm();
    max_jump_t = std::max(max_jump_t, 4.0 * predicted_step_t + 0.05);
  }
  double max_jump_rot_deg = kMaxPoseJumpRotationDeg;
  if (pnp_inliers >= kStrongPnPInliers) {
    max_jump_t *= 2.0;
    max_jump_rot_deg *= 2.0;
  }

  if (pose_jump_t > max_jump_t || pose_jump_rot_deg > max_jump_rot_deg) {
    std::cout << "[Frontend] Reject pose jump | dT=" << pose_jump_t << " (max "
              << max_jump_t << ")"
              << " | dR=" << pose_jump_rot_deg << " deg (max "
              << max_jump_rot_deg << ")" << std::endl;
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────
// KEYFRAME INSERTION
// ─────────────────────────────────────────────────────────────────

bool VisualFrontend::shouldInsertKeyFrame(Frame::Ptr current_frame) const {
  if (frames_since_last_kf_ < kMinFramesBetweenKF)
    return false;

  size_t tracked = countTrackedMapPoints(current_frame);
  if (static_cast<int>(tracked) < kMinTrackedMapPoints)
    return (static_cast<int>(tracked) >= kMinTrackedForNewKF);

  Pose3d T_kf_curr =
      last_keyframe_->getPose().inverse() * current_frame->getPose();
  if (T_kf_curr.translation().norm() > kMinBaseline)
    return true;

  Eigen::AngleAxisd aa(T_kf_curr.linear());
  if (aa.angle() * 180.0 / M_PI > kMinRotationDeg)
    return true;

  return false;
}

void VisualFrontend::insertKeyFrame(Frame::Ptr current_frame) {
  KeyFrame::Ptr new_kf = KeyFrame::create(current_frame);
  new_kf->registerMapPointObservations();
  map_->addKeyFrame(new_kf);
  triangulateNewPoints(last_keyframe_, new_kf);
  new_kf->updateCovisibility();
  last_keyframe_->updateCovisibility();
  new_kf->syncMapPointsToFrame(current_frame);

  Pose3d relative_pose =
      last_keyframe_->getPose().inverse() * new_kf->getPose();
  emitBackendOutput(false, relative_pose, new_kf->getPose(),
                    current_frame->getTimestamp(), last_covariance_);

  reference_keyframe_ = new_kf;
  last_keyframe_ = new_kf;
  frames_since_last_kf_ = 0;
  map_->cleanBadMapPoints();

  std::cout << "[Frontend] ★ New KF " << new_kf->getId()
            << " | KF MPs: " << new_kf->countMapPoints()
            << " | Map MPs: " << map_->numMapPoints() << std::endl;
}

void VisualFrontend::emitBackendOutput(
    bool is_first, const Pose3d &relative, const Pose3d &estimate,
    double timestamp, const Eigen::Matrix<double, 6, 6> &covariance) {
  FrontendOutput output;
  output.is_first_keyframe = is_first;
  output.relative_pose = relative;
  output.initial_estimate = estimate;
  output.timestamp = timestamp;
  output.pose_covariance = covariance;
  if (is_first)
    output.prior_pose = relative;
  pending_output_ = output;
}

void VisualFrontend::rollbackFailedInitialization(KeyFrame::Ptr curr_kf) {
  for (auto &mp : curr_kf->accessMapPoints()) {
    if (!mp)
      continue;
    mp->setBad();
    map_->removeMapPoint(mp);
    mp = nullptr;
  }
  for (auto &mp : last_keyframe_->accessMapPoints()) {
    if (mp && mp->isBad_)
      mp = nullptr;
  }
  map_->removeKeyFrame(curr_kf);
}

// ─────────────────────────────────────────────────────────────────
// RELOCALIZATION (LOST state)
// ─────────────────────────────────────────────────────────────────

bool VisualFrontend::tryRelocalize(Frame::Ptr current_frame) {
  if (!current_frame || map_->numKeyFrames() == 0)
    return false;

  // Collect candidate keyframes sorted by recency (newest first)
  std::vector<KeyFrame::Ptr> candidates(map_->getAllKeyFrames().begin(),
                                        map_->getAllKeyFrames().end());
  std::sort(candidates.begin(), candidates.end(),
            [](const KeyFrame::Ptr &a, const KeyFrame::Ptr &b) {
              return a->getTimestamp() > b->getTimestamp();
            });

  // Try matching against each candidate keyframe
  const size_t max_candidates = std::min(candidates.size(), size_t(5));
  for (size_t c = 0; c < max_candidates; c++) {
    KeyFrame::Ptr kf = candidates[c];
    if (!kf || kf->getDescriptors().empty())
      continue;

    auto matches = feature_matcher_->match(kf->getDescriptors(),
                                           current_frame->getDescriptors(),
                                           kMatchRatioThreshold);
    if (matches.size() < kMinMatchesForInit)
      continue;

    // Associate MapPoints from this keyframe to the current frame
    current_frame->ensureMapPointVectorSized(
        current_frame->getKeypoints().size());
    // Clear previous associations for a fresh attempt
    for (auto &mp : current_frame->accessMapPoints())
      mp = nullptr;

    size_t associated = 0;
    for (const auto &m : matches) {
      size_t kf_idx = static_cast<size_t>(m.queryIdx);
      size_t fr_idx = static_cast<size_t>(m.trainIdx);
      if (fr_idx >= current_frame->accessMapPoints().size())
        continue;
      if (kf_idx < kf->getMapPoints().size()) {
        MapPoint::Ptr mp = kf->getMapPoints()[kf_idx];
        if (mp && !mp->isBad_) {
          current_frame->accessMapPoints()[fr_idx] = mp;
          associated++;
        }
      }
    }

    if (static_cast<int>(associated) < kMinPnPInliers)
      continue;

    // Set an initial pose guess from the candidate keyframe
    current_frame->setPose(kf->getPose());

    int pnp_inliers = 0;
    if (pose_estimator_->estimateRefined(current_frame, &pnp_inliers) &&
        pnp_inliers >= kMinPnPInliers) {
      // Update reference to the keyframe that relocated us
      reference_keyframe_ = kf;
      last_keyframe_ = kf;
      frames_since_last_kf_ = 0;
      std::cout << "[Frontend] Reloc via KF " << kf->getId()
                << " | inliers=" << pnp_inliers << " | Pose: "
                << current_frame->getPose().translation().transpose()
                << std::endl;
      return true;
    }
  }

  std::cout << "[Frontend] Relocalization attempt " << lost_frames_ << "/"
            << kMaxRelocFrames << " failed" << std::endl;
  return false;
}

void VisualFrontend::resetToInitializing() {
  // Preserve the last known good pose as the starting point
  Pose3d restart_pose = last_good_pose_;

  // TODO: Implement KeyFrame culling and sliding-window pruning so the map
  //       doesn't grow unbounded during long sequences. Currently the only
  //       cleanup is this full reset. See Map::getKeyFramesInWindow() (unused)
  //       and consider culling redundant KFs whose MapPoints are well-observed
  //       by other KFs (ORB-SLAM2-style).
  map_ = std::make_shared<Map>();

  // Reset all state
  last_frame_ = nullptr;
  last_keyframe_ = nullptr;
  reference_keyframe_ = nullptr;
  current_frame_ = nullptr;
  has_velocity_ = false;
  velocity_ = Pose3d::Identity();
  consecutive_failures_ = 0;
  lost_frames_ = 0;
  frames_since_last_kf_ = 0;
  init_attempts_ = 0;

  // Transition back to NO_IMAGES_YET — the next frame will
  // start a fresh map segment anchored at the last known pose
  stage_ = Stage::NO_IMAGES_YET;

  std::cout << "[Frontend] Reset complete. Last known pose: "
            << restart_pose.translation().transpose() << std::endl;
}

void VisualFrontend::refreshInitializationReference(Frame::Ptr current_frame) {
  // Drop the stale init KF (it has no map points and is just a descriptor
  // anchor that's no longer useful).
  if (last_keyframe_)
    map_->removeKeyFrame(last_keyframe_);

  // Re-anchor at last_good_pose_. We have no tracking signal during
  // INITIALIZING so we can't know how far the camera actually moved; the
  // backend's other sensors are expected to correct any anchor error.
  current_frame->setPose(last_good_pose_);
  last_keyframe_ = KeyFrame::create(current_frame);
  last_keyframe_->registerMapPointObservations();
  reference_keyframe_ = last_keyframe_;
  map_->addKeyFrame(last_keyframe_);
  last_frame_ = current_frame;
  frames_since_last_kf_ = 0;

  std::cout << "[Frontend Init] Refreshed reference → KF "
            << last_keyframe_->getId() << " at "
            << last_good_pose_.translation().transpose() << std::endl;
}

// ─────────────────────────────────────────────────────────────────
// TRIANGULATION
// ─────────────────────────────────────────────────────────────────

void VisualFrontend::triangulateNewPoints(KeyFrame::Ptr kf1,
                                          KeyFrame::Ptr kf2) {
  if (!kf1 || !kf2)
    return;
  if (kf1->getDescriptors().empty() || kf2->getDescriptors().empty())
    return;

  auto good_matches = feature_matcher_->match(
      kf1->getDescriptors(), kf2->getDescriptors(), kMatchRatioThreshold);
  triangulateNewPoints(kf1, kf2, good_matches);
}

void VisualFrontend::triangulateNewPoints(
    KeyFrame::Ptr kf1, KeyFrame::Ptr kf2,
    const std::vector<cv::DMatch> &good_matches) {
  if (!kf1 || !kf2 || good_matches.empty())
    return;

  std::vector<cv::Point2f> pts1, pts2;
  std::vector<cv::DMatch> matches_to_triangulate;
  for (const auto &m : good_matches) {
    if (m.queryIdx < 0 || m.trainIdx < 0)
      continue;
    const size_t idx1 = static_cast<size_t>(m.queryIdx);
    const size_t idx2 = static_cast<size_t>(m.trainIdx);
    if (idx1 >= kf1->getMapPoints().size() ||
        idx2 >= kf2->getMapPoints().size())
      continue;
    if (idx1 >= kf1->getKeypoints().size() ||
        idx2 >= kf2->getKeypoints().size())
      continue;

    bool has1 = (kf1->getMapPoints()[idx1] != nullptr);
    bool has2 = (kf2->getMapPoints()[idx2] != nullptr);

    if (has1 && has2)
      continue;

    // Propagate existing MapPoint to the other keyframe
    if (has1) {
      MapPoint::Ptr mp = kf1->getMapPoints()[idx1];
      if (mp && !mp->isBad_) {
        mp->addObservation(kf2, idx2);
        kf2->accessMapPoints()[idx2] = mp;
      }
      continue;
    }
    if (has2) {
      MapPoint::Ptr mp = kf2->getMapPoints()[idx2];
      if (mp && !mp->isBad_) {
        mp->addObservation(kf1, idx1);
        kf1->accessMapPoints()[idx1] = mp;
      }
      continue;
    }

    // Neither keyframe has a MapPoint → needs triangulation
    pts1.push_back(kf1->getKeypoints()[idx1].pt);
    pts2.push_back(kf2->getKeypoints()[idx2].pt);
    matches_to_triangulate.push_back(m);
  }
  if (pts1.empty())
    return;

  Pose3d T_w_1 = kf1->getPose();
  // T_2_1 transforms points from cam1 frame to cam2 frame
  Pose3d T_2_1 = kf2->getPose().inverse() * kf1->getPose();

  cv::Mat R_cv, t_cv;
  pose3dToCv(T_2_1, R_cv, t_cv);

  std::vector<cv::Point3f> points_3d;
  pose_estimator_->triangulate(pts1, pts2, K_, R_cv, t_cv, points_3d);

  Eigen::Vector3d cam1_w = kf1->getPose().translation();
  Eigen::Vector3d cam2_w = kf2->getPose().translation();

  size_t created = 0;
  for (size_t i = 0;
       i < points_3d.size() && i < matches_to_triangulate.size(); i++) {
    if (!isTriangulatedPointValid(points_3d[i], R_cv, t_cv, cam1_w, cam2_w,
                                  T_w_1, kMaxTriangulationDist,
                                  kMinParallaxDeg))
      continue;

    const auto &p = points_3d[i];
    Eigen::Vector3d p_world = T_w_1 * Eigen::Vector3d(p.x, p.y, p.z);
    MapPoint::Ptr mp = MapPoint::create(p_world);

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

// ─────────────────────────────────────────────────────────────────
// UTILITIES
// ─────────────────────────────────────────────────────────────────

size_t VisualFrontend::countTrackedMapPoints(Frame::Ptr frame) const {
  size_t count = 0;
  for (const auto &mp : frame->getMapPoints()) {
    if (mp && !mp->isBad_)
      count++;
  }
  return count;
}

Frame::Ptr VisualFrontend::getLatestFrame() const { return last_frame_; }

std::optional<FrontendOutput> VisualFrontend::consumeBackendOutput() {
  auto out = std::move(pending_output_);
  pending_output_.reset();
  return out;
}
KeyFrame::Ptr VisualFrontend::getLatestKeyFrame() const {
  return last_keyframe_;
}

// ─────────────────────────────────────────────────────────────────
// EXTERNAL INTEGRATION (MIMOSA + DVL)
// ─────────────────────────────────────────────────────────────────

void VisualFrontend::setInitialPose(const Pose3d &T_w_cam0) {
  if (stage_ != Stage::NO_IMAGES_YET) {
    return;  // First KF already created; ignore.
  }
  initial_pose_override_ = T_w_cam0;
}

void VisualFrontend::setBodyCamExtrinsic(const Pose3d &T_body_cam) {
  T_body_cam_ = T_body_cam;
}

void VisualFrontend::pushDvlSample(const DvlSample &sample) {
  dvl_buf_.push_back(sample);
  // Evict samples older than dvl_buf_max_age_s_ relative to the newest.
  const double newest = sample.t;
  while (!dvl_buf_.empty() &&
         (newest - dvl_buf_.front().t) > dvl_buf_max_age_s_) {
    dvl_buf_.pop_front();
  }
}

void VisualFrontend::setExternalPoseHint(const ExternalPoseHint &hint) {
  pending_hint_ = hint;
}

bool VisualFrontend::buildDvlPrior(double t0, double t1,
                                   const Pose3d &T_w_cam_prev,
                                   Eigen::Vector3d *out_dp_world,
                                   Eigen::Matrix3d *out_cov_world) const {
  if (!out_dp_world || !out_cov_world)
    return false;
  if (t1 <= t0 || dvl_buf_.size() < 2)
    return false;

  // Trapezoidal integration of body-frame velocity over [t0, t1].
  // Walks through the buffer, clipping the first/last segments to the
  // exact interval and skipping segments where either endpoint has lost
  // bottom_lock.
  Eigen::Vector3d dp_body = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov_body = Eigen::Matrix3d::Zero();
  double covered = 0.0;

  for (size_t i = 0; i + 1 < dvl_buf_.size(); ++i) {
    const auto &a = dvl_buf_[i];
    const auto &b = dvl_buf_[i + 1];
    const double seg_t0 = std::max(a.t, t0);
    const double seg_t1 = std::min(b.t, t1);
    if (seg_t1 <= seg_t0)
      continue;
    if (!a.bottom_lock || !b.bottom_lock)
      continue;

    // Linear interpolation of velocity at the clipped endpoints.
    auto lerp_v = [&](double t) {
      const double u = (t - a.t) / std::max(1e-9, b.t - a.t);
      return a.v_body + u * (b.v_body - a.v_body);
    };
    const Eigen::Vector3d v0 = lerp_v(seg_t0);
    const Eigen::Vector3d v1 = lerp_v(seg_t1);
    const double dt = seg_t1 - seg_t0;

    dp_body += 0.5 * (v0 + v1) * dt;
    // Covariance accumulates as Σ cov_body * dt² (independent intervals).
    cov_body += 0.5 * (a.cov_body + b.cov_body) * dt * dt;
    covered += dt;
  }

  const double interval = t1 - t0;
  if (covered < kMinDvlIntervalCoverage * interval)
    return false;

  // Rotate body-frame displacement and covariance into the world frame.
  // T_w_body_prev = T_w_cam_prev * T_cam_body = T_w_cam_prev * T_body_cam_⁻¹.
  const Pose3d T_w_body_prev = T_w_cam_prev * T_body_cam_.inverse();
  const Eigen::Matrix3d R_w_body = T_w_body_prev.linear();
  *out_dp_world = R_w_body * dp_body;
  *out_cov_world = R_w_body * cov_body * R_w_body.transpose();

  // Floor the covariance to avoid numerical singularity from zero motion.
  const double floor = 1e-6;
  for (int k = 0; k < 3; ++k)
    (*out_cov_world)(k, k) = std::max((*out_cov_world)(k, k), floor);

  return true;
}

void VisualFrontend::lockScale() {
  if (scale_locked_ || cum_vo_dist_ <= 0.0 || cum_dvl_dist_ <= 0.0)
    return;

  const double s = cum_dvl_dist_ / cum_vo_dist_;
  if (!std::isfinite(s) || s <= 0.0)
    return;

  // Pivot: world origin (or initial_pose_override_, which has been folded
  // into last_good_pose_'s history). Since the first KF was placed at
  // last_good_pose_ and that pose is the world reference, use it as p0.
  // For the simple case where the world frame == first KF frame, p0 is the
  // first KF's translation; in either case, all pose translations and map
  // point positions are scaled around it.
  Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
  // Find the oldest KF as the pivot (the first one created).
  KeyFrame::Ptr pivot;
  for (const auto &kf : map_->getAllKeyFrames()) {
    if (!pivot || kf->getId() < pivot->getId())
      pivot = kf;
  }
  if (pivot)
    p0 = pivot->getPose().translation();

  for (const auto &kf : map_->getAllKeyFrames()) {
    Pose3d T = kf->getPose();
    T.translation() = p0 + s * (T.translation() - p0);
    kf->setPose(T);
  }
  for (const auto &mp : map_->getAllMapPoints()) {
    if (!mp)
      continue;
    mp->position_ = p0 + s * (mp->position_ - p0);
  }

  // Update transient state mirroring KF/MapPoint scale.
  if (last_frame_) {
    Pose3d T = last_frame_->getPose();
    T.translation() = p0 + s * (T.translation() - p0);
    last_frame_->setPose(T);
  }
  last_good_pose_.translation() =
      p0 + s * (last_good_pose_.translation() - p0);
  velocity_.translation() *= s;

  scale_locked_ = true;

  std::cout << "[Frontend] Scale lock: s=" << s
            << " (cum_vo=" << cum_vo_dist_
            << ", cum_dvl=" << cum_dvl_dist_ << ")" << std::endl;
}

} // namespace frontend
