#include "frontend/FeatureMatcher.hpp"
#include <iostream>
#include <gtsam/geometry/Point3.h>

namespace frontend {

FeatureMatcher::FeatureMatcher(const std::string& matcher_type, float ratio_thresh) 
    : ratio_thresh_(ratio_thresh) {
    
    if (matcher_type == "NORM_L2") {
        matcher_ = cv::DescriptorMatcher::create("BruteForce");
    } else {
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
    }
}

std::vector<cv::DMatch> FeatureMatcher::match(Frame::Ptr frame1, Frame::Ptr frame2) {
    std::vector<cv::DMatch> good_matches;
    
    // Check if frames exist and have descriptors
    if (!frame1 || !frame2 || frame1->getDescriptors().empty() || frame2->getDescriptors().empty()) {
        return good_matches;
    }

    std::vector<std::vector<cv::DMatch>> knn_matches;
    
    // k = 2 for Ratio Test
    try {
        matcher_->knnMatch(frame1->getDescriptors(), frame2->getDescriptors(), knn_matches, 2);
    } catch (const cv::Exception& e) {
        std::cerr << "[FeatureMatcher] Error during matching: " << e.what() << std::endl;
        return good_matches;
    }

    // Apply Lowe's Ratio Test
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i].size() < 2) continue;
        
        if (knn_matches[i][0].distance < ratio_thresh_ * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    return good_matches;
}

int FeatureMatcher::matchProjection(Frame::Ptr current, const std::vector<MapPoint::Ptr>& local_map_points, float radius) {
    if (!current || local_map_points.empty()) return 0;
    
    int n_matches = 0;
    
    const cv::Mat& K = current->getCamera();
    if (K.empty()) {
        std::cerr << "[FeatureMatcher] Camera intrinsics missing in Frame!" << std::endl;
        return 0;
    }
    
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    
    // Get current pose (T_w_c estimate)
    gtsam::Pose3 T_w_c = current->getPoseGuess(); 
    // We need T_c_w to project world points to camera
    gtsam::Pose3 T_c_w = T_w_c.inverse();
    
    // For each MapPoint
    for (const auto& mp : local_map_points) {
        if (!mp || mp->isBad()) continue;
        
        // 1. Transform Point to Camera Frame
        gtsam::Point3 p_w(mp->getPosition());
        gtsam::Point3 p_c = T_c_w.transformFrom(p_w);
        
        // Check if in front of camera
        if (p_c.z() <= 0.1) continue;
        
        // 2. Project to Image Plane
        double u = fx * p_c.x() / p_c.z() + cx;
        double v = fy * p_c.y() / p_c.z() + cy;
        
        // Check bounds
        if (u < 0 || u >= current->getImage().cols || v < 0 || v >= current->getImage().rows) continue;
        
        // 3. Search for features in radius
        // Optimization: In real SLAM, use a grid or spatial hash.
        // Here: simple linear search (slow but correct for prototype)
        
        float best_dist = 256.0f; // Max Hamming distance
        int best_idx = -1;
        
        const std::vector<cv::KeyPoint>& kps = current->getKeypoints();
        const cv::Mat& des = current->getDescriptors();
        cv::Mat mp_des = mp->getDescriptor();
        
        for (size_t i = 0; i < kps.size(); i++) {
            // Already matched?
            if (current->getObservation(i)) continue;
            
            float dx = kps[i].pt.x - u;
            float dy = kps[i].pt.y - v;
            float dist_sq = dx*dx + dy*dy;
            
            if (dist_sq < radius * radius) {
                // Compute Descriptor Distance
                // Assuming Hamming for ORB
                int dist = 0;
                // Manual hamming or cv::norm
                 // Need to handle different descriptor types generically? 
                 // Assuming ORB (binary) for now as typical in these requests.
                 // Using cv::norm with NORM_HAMMING
                  dist = cv::norm(mp_des, des.row(i), cv::NORM_HAMMING);
                 
                 if (dist < best_dist) {
                     best_dist = dist;
                     best_idx = i;
                 }
            }
        }
        
        // 4. Threshold and Add Match
        // Hardcoded threshold 50 for Hamming
        if (best_idx != -1 && best_dist < 50.0f) {
            current->addObservation(best_idx, mp);
            // Also add observation to MapPoint? MapPoint->addObservation(current, best_idx); 
            // Note: Frame is transient, MapPoint usually tracks KeyFrames. 
            // But for tracking optimization, we just need the link in Frame.
            n_matches++;
        }
    }
    
    return n_matches;
}

} // namespace frontend
