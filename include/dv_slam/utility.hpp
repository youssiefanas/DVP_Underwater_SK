// convert OpenCV R,t to Eigen Isometry3d (Pose3d)
#pragma once

#include "frontend/Pose3d.hpp"
#include <opencv2/core.hpp>

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
