// convert OpenCV R,t to Eigen Isometry3d (Pose3d)
#pragma once

#include "frontend/Pose3d.hpp"
#include <opencv2/core.hpp>

/// Convert a Pose3d to OpenCV R (3×3, CV_64F) and t (3×1, CV_64F).
inline void pose3dToCv(const frontend::Pose3d &pose, cv::Mat &R_out,
                       cv::Mat &t_out) {
    const Eigen::Matrix3d R_eig = pose.linear();
    const Eigen::Vector3d t_eig = pose.translation();
    R_out = cv::Mat(3, 3, CV_64F);
    t_out = cv::Mat(3, 1, CV_64F);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            R_out.at<double>(i, j) = R_eig(i, j);
        t_out.at<double>(i) = t_eig(i);
    }
}

inline frontend::Pose3d cvToPose3d(const cv::Mat& R, const cv::Mat& t) {
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
    return frontend::makePose(R_eigen, t_eigen);
}
