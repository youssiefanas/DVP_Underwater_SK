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

    if (E.empty()) {
        return false;
    }

    int inliers = cv::recoverPose(E, points_prev, points_curr, K, R, t, mask);

    if (inliers < 5) {
        return false;
    }

    return true;
}

bool PoseEstimator::estimateRefined(Frame::Ptr frame) {
    if (!frame) return false;
    
    const cv::Mat& K = frame->getCamera();
    if (K.empty()) return false;
    
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    
    const auto& observations = frame->getObservations();
    const auto& keypoints = frame->getKeypoints();
    
    for (size_t i = 0; i < observations.size(); i++) {
        if (observations[i]) {
             Eigen::Vector3d pos = observations[i]->getPosition();
             object_points.emplace_back(pos.x(), pos.y(), pos.z());
             image_points.push_back(keypoints[i].pt);
        }
    }
    
    if (object_points.size() < 4) {
        std::cout << "[PoseEstimator] Not enough observations for PnP: " << object_points.size() << std::endl;
        return false;
    }
    
    // Initial guess from frame
    gtsam::Pose3 prior = frame->getPoseGuess();
    // GTSAM Pose3 -> OpenCV R, t
    // T_w_c (World to Camera) OR T_c_w (Camera to World)?
    // solvePnP finds T_c_w (World point transformed to Camera frame)
    // Frame::pose is usually T_w_c (Camera pose in World). 
    // So we need inverse of pose_guess if it is T_w_c.
    // Let's assume setPose/getPose corresponds to T_w_c (Robot Pose).
    
    gtsam::Pose3 T_c_w = prior.inverse();
    gtsam::Matrix3 R_gtsam = T_c_w.rotation().matrix();
    gtsam::Vector3 t_gtsam = T_c_w.translation();
    
    cv::Mat rvec, tvec;
    cv::Mat R_cv(3, 3, CV_64F);
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            R_cv.at<double>(i,j) = R_gtsam(i,j);
            
    cv::Rodrigues(R_cv, rvec);
    tvec = (cv::Mat_<double>(3, 1) << t_gtsam.x(), t_gtsam.y(), t_gtsam.z());
    
    cv::Mat inliers;
    bool success = cv::solvePnPRansac(object_points, image_points, K, cv::Mat(),
                                      rvec, tvec, 
                                      true, // useExtrinsicGuess
                                      100, // iterations
                                      2.0, // reprojection error
                                      0.99, // confidence
                                      inliers);
                                      
    if (success) {
        // Convert back to T_w_c
        cv::Mat R_final;
        cv::Rodrigues(rvec, R_final);
        
        Eigen::Matrix3d R_eigen;
        Eigen::Vector3d t_eigen;
        
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) R_eigen(i,j) = R_final.at<double>(i,j);
            t_eigen(i) = tvec.at<double>(i);
        }
        
        gtsam::Rot3 rot(R_eigen);
        gtsam::Point3 trans(t_eigen);
        
        gtsam::Pose3 T_c_w_final(rot, trans);
        gtsam::Pose3 T_w_c_final = T_c_w_final.inverse();
        
        // Update Frame Pose
        frame->setPose(T_w_c_final);
        
        // Output Quality Assessment Info?
        // std::cout << "PnP Inliers: " << inliers.rows << "/" << object_points.size() << std::endl;
        
        // Remove outliers from observations?
        // TODO: Mark outliers in Frame so they are not used for mapping
        // For now, we trust RANSAC output for pose, but data association cleanup is nice to have.
        
        return true;
    }
    
    return false;
}

} // namespace frontend
