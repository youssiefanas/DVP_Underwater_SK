#include "frontend/PoseEstimator.hpp"
#include <iostream>
#include <gtsam/geometry/Pose3.h> // For pose conversion if needed
// Or just use OpenCV R, t directly and convert to GTSAM at the end

namespace frontend {

PoseEstimator::PoseEstimator() {}

bool PoseEstimator::estimate(const std::vector<cv::Point2f>& points_prev,
                             const std::vector<cv::Point2f>& points_curr,
                             const cv::Mat& K,
                             cv::Mat& R,
                             cv::Mat& t,
                             cv::Mat& mask) 
{
    if (points_prev.size() < 5 || points_curr.size() < 5) {
        // std::cout << "[PoseEstimator] Not enough points for Essential Matrix (needs >= 5)." << std::endl;
        return false;
    }

    double focal = K.at<double>(0, 0);
    cv::Point2d pp(K.at<double>(0, 2), K.at<double>(1, 2));
    
    cv::Mat E = cv::findEssentialMat(points_prev, points_curr, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    // E is a 3x3 matrix representing the Essential Matrix for the relative pose between the two frames

    if (E.empty()) {
        return false;
    }

    int inliers = cv::recoverPose(E, points_prev, points_curr, K, R, t, mask);
    

    if (inliers < 5) {
        return false;
    }

    return true;
}
bool PoseEstimator::triangulate(const std::vector<cv::Point2f>& points_prev,
                                const std::vector<cv::Point2f>& points_curr,
                                const cv::Mat& K,
                                const cv::Mat& R,
                                const cv::Mat& t,
                                std::vector<cv::Point3f>& points_3d)
                                {
    cv::Mat T1 = cv::Mat::eye(3,4,CV_64F);
    cv::Mat T2 = cv::Mat::zeros(3,4,CV_64F);
    R.copyTo(T2(cv::Rect(0,0,3,3)));
    t.copyTo(T2(cv::Rect(3,0,1,3)));
    cv::Mat P1 = K * T1;
    cv::Mat P2 = K * T2;
    cv::Mat points_4d; // for triangulation, x,y,z,w
    cv::triangulatePoints(P1, P2, points_prev, points_curr, points_4d);

    // convert points_4d to points_3d
    points_3d.clear();
    for (int i = 0; i < points_4d.cols; i++) {
        cv::Point3f point(points_4d.at<float>(0, i), points_4d.at<float>(1, i), points_4d.at<float>(2, i));
        points_3d.push_back(point);
    }
    return true;
    
                                    
                                    

                                    
                                }
}