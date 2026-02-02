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
