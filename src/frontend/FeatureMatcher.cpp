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
}