#include "frontend/FeatureExtractor.hpp"    
#include <iostream>
namespace frontend {

FeatureExtractor::FeatureExtractor(const ORBParams& params) {
    // Initialize ORB with parameters from struct
    orb_ = cv::ORB::create(
        params.n_features,
        params.scale_factor,
        params.n_levels,
        params.edge_threshold,
        params.first_level,
        params.wta_k,
        static_cast<cv::ORB::ScoreType>(params.score_type),
        params.patch_size,
        params.fast_threshold
    ); 
    std::cout<<"ORB initialized with parameters: "<<params.n_features<<std::endl;
}

void FeatureExtractor::extract(Frame& frame) {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Detect and Compute
    orb_->detectAndCompute(frame.getImage(), cv::noArray(), keypoints, descriptors);

    // Store in Frame
    frame.setFeatures(keypoints, descriptors);

    // Debug: Print number of features found
    // std::cout << "[FeatureExtractor] Extracted " << keypoints.size() << " features from Frame " << frame.getId() << std::endl;
}

} // namespace frontend
