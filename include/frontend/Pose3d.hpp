#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace frontend {

/// SE(3) rigid-body transform — replaces gtsam::Pose3 in the frontend.
using Pose3d = Eigen::Isometry3d;


/// Construct a Pose3d from a 3x3 rotation matrix and a 3x1 translation.
inline Pose3d makePose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t) {
  Pose3d pose = Pose3d::Identity();
  pose.linear() = R;
  pose.translation() = t;
  return pose;
}

} // namespace frontend
