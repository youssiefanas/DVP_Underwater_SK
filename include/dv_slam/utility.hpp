#pragma once
// convert OpenCV R,t to GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

inline gtsam::Pose3 cvToGtsam(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat R_mat, t_mat;
    if(R.type() != CV_64F) R.convertTo(R_mat, CV_64F);
    else R_mat = R;
    if(t.type() != CV_64F) t.convertTo(t_mat, CV_64F);
    else t_mat = t;

    Eigen::Matrix3d R_eigen;
    Eigen::Vector3d t_eigen;
    
for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            R_eigen(i,j) = R_mat.at<double>(i,j);
        }
        t_eigen(i) = t_mat.at<double>(i);
    }
    return gtsam::Pose3(gtsam::Rot3(R_eigen), gtsam::Point3(t_eigen));
}

// Explicit identity pose (avoid relying on default constructors).
inline gtsam::Pose3 identityPose() {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
                        gtsam::Point3(0.0, 0.0, 0.0));
}

// ── GTSAM ABI workaround ──────────────────────────────────────────
// Pose3::translation() reads from the wrong memory offset due to an ABI
// mismatch between GTSAM headers and the installed library (sizeof(Rot3)
// differs). Rotation extraction works because Rot3 is at offset 0.
//
// RULE: Never call Pose3::inverse(), Pose3::compose(), or operator* on
//       Pose3 objects — the result's translation is garbage.
//       Instead, extract R and t from user-created Pose3 objects (whose
//       translation() IS correct) and compute relative poses manually.

/// Safe extraction of rotation + translation from a USER-CREATED Pose3.
/// (User-created = built via inline cvToGtsam / identityPose, not from
///  GTSAM library ops like inverse() or compose().)
struct Rt {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};

inline Rt extractRt(const gtsam::Pose3 &pose) {
    return {pose.rotation().matrix(), pose.translation()};
}

/// Compute T_2_1 = T_w_2^{-1} * T_w_1 WITHOUT calling GTSAM inverse/compose.
inline Rt relativeRt(const gtsam::Pose3 &T_w_1, const gtsam::Pose3 &T_w_2) {
    Rt p1 = extractRt(T_w_1);
    Rt p2 = extractRt(T_w_2);
    Eigen::Matrix3d R_rel = p2.R.transpose() * p1.R;
    Eigen::Vector3d t_rel = p2.R.transpose() * (p1.t - p2.t);
    return {R_rel, t_rel};
}

/// Build a GTSAM Pose3 relative pose from two user-created Pose3s.
/// Safe alternative to `T2.inverse() * T1`.
inline gtsam::Pose3 safeRelativePose(const gtsam::Pose3 &T_w_from,
                                     const gtsam::Pose3 &T_w_to) {
    Rt rel = relativeRt(T_w_from, T_w_to);
    return gtsam::Pose3(gtsam::Rot3(rel.R), gtsam::Point3(rel.t));
}
