#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace frontend {

/// Unary projection factor: optimizes a single Pose3 while holding the 3D
/// landmark fixed.  This is the standard "motion-only BA" factor used in
/// ORB-SLAM-style systems.
class FixedLandmarkProjectionFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  FixedLandmarkProjectionFactor(const gtsam::Point2 &measured,
                                const gtsam::SharedNoiseModel &model,
                                gtsam::Key pose_key,
                                const gtsam::Point3 &landmark,
                                const std::shared_ptr<gtsam::Cal3_S2> &K)
      : Base(model, pose_key), measured_(measured), landmark_(landmark),
        K_(K) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      gtsam::OptionalMatrixType H = nullptr) const override {
    try {
      gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pose, *K_);
      gtsam::Point2 projected = camera.project(landmark_, H, {}, {});
      return projected - measured_;
    } catch (gtsam::CheiralityException &) {
      if (H)
        *H = gtsam::Matrix::Zero(2, 6);
      return gtsam::Vector2(2.0 * K_->fx(), 2.0 * K_->fx());
    }
  }

private:
  gtsam::Point2 measured_;
  gtsam::Point3 landmark_;
  std::shared_ptr<gtsam::Cal3_S2> K_;
};

} // namespace frontend
