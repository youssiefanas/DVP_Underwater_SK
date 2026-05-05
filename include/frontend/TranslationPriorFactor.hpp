#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace frontend {

/// Unary translation-only prior on a Pose3: pulls the pose's translation
/// component toward a target while leaving rotation unconstrained.
///
/// Used in motion-only BA to enforce a metric translation derived from
/// DVL velocity integration. The error is the world-frame translation
/// difference; the rotation block of the Jacobian is zero.
class TranslationPriorFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  TranslationPriorFactor(gtsam::Key pose_key,
                         const gtsam::Point3 &target_t_world,
                         const gtsam::SharedNoiseModel &model)
      : Base(model, pose_key), target_(target_t_world) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      gtsam::OptionalMatrixType H = nullptr) const override {
    if (H) {
      // Pose3 tangent ordering in GTSAM is [rot(3), trans(3)].
      // d(t_world)/d(xi_rot) = -R * skew(t_local)  → for the local-translation
      // perturbation in a Pose3 retract, the Jacobian of the world translation
      // w.r.t. the 6-vector xi is [0_3x3 , R].
      *H = gtsam::Matrix::Zero(3, 6);
      H->block<3, 3>(0, 3) = pose.rotation().matrix();
    }
    return pose.translation() - target_;
  }

private:
  gtsam::Point3 target_;
};

} // namespace frontend
