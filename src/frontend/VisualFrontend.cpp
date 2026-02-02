#include "frontend/VisualFrontend.hpp"
#include "frontend/PoseEstimator.hpp"
#include <iostream>

namespace frontend {

VisualFrontend::VisualFrontend(const ORBParams& params, const cv::Mat& K) : stage_(Stage::NO_IMAGES_YET) {
    feature_extractor_ = std::make_shared<FeatureExtractor>(params);
    viewer_ = std::make_shared<Viewer>();
    feature_matcher_ = std::make_shared<FeatureMatcher>(params.matcher_type);
    pose_estimator_ = std::make_shared<PoseEstimator>();
    
    // Initialize New Components
    map_manager_ = std::make_shared<LocalMapManager>();
    keyframe_selector_ = std::make_shared<KeyframeSelector>();
    
    K_ = K.clone();
    
    // Init state
    last_pose_ = gtsam::Pose3();
    velocity_ = gtsam::Pose3();
}

bool VisualFrontend::handleImage(
    const cv::Mat& gray_image,
    double timestamp)
{
    // Create frame (data ownership)
    Frame::Ptr frame = Frame::createFrame(gray_image, timestamp);
    frame->setCamera(K_);

    // Feature extraction (annotation)
    extractFeatures(frame);
    
    // Core frontend logic
    bool success = process(frame);
    
    // Visualization
    if (viewer_) {
        cv::Mat img_out;
        cv::drawKeypoints(frame->getImage(), frame->getKeypoints(), img_out);
        
        // Draw tracked points in green
        const auto& obs = frame->getObservations();
        const auto& kps = frame->getKeypoints();
        for(size_t i=0; i<obs.size(); i++) {
            if(obs[i]) {
                 cv::circle(img_out, kps[i].pt, 4, cv::Scalar(0, 255, 0), 2);
            }
        }
        
        viewer_->show(img_out);
    }
    
    return success;
}

void VisualFrontend::extractFeatures(Frame::Ptr frame)
{
    feature_extractor_->extract(*frame);
}

bool VisualFrontend::process(Frame::Ptr current_frame) {
    
    // --- 1. Initialization Case ---
    if (stage_ == Stage::NO_IMAGES_YET) {
        std::cout << "[Frontend] First frame received. Initializing..." << std::endl;
        last_frame_ = current_frame;
        // Assume Identity for first frame
        current_frame->setPose(gtsam::Pose3());
        last_pose_ = gtsam::Pose3();
        
        stage_ = Stage::INITIALIZED;
        return false;
    }
    else if (stage_ == Stage::INITIALIZED) {
        // Try to initialize map with 2nd frame
        std::vector<cv::DMatch> matches = feature_matcher_->match(last_frame_, current_frame);
        
        if (matches.size() < 20) {
            std::cout << "[Frontend] Not enough matches for init. Retrying..." << std::endl;
            last_frame_ = current_frame; // Shift
            return false;
        }
        
        std::vector<cv::Point2f> points_prev, points_curr;
        for (const auto& m : matches) {
            points_prev.push_back(last_frame_->getKeypoints()[m.queryIdx].pt);
            points_curr.push_back(current_frame->getKeypoints()[m.trainIdx].pt);
        }
        
        cv::Mat R, t, mask;
        if (pose_estimator_->estimate(points_prev, points_curr, K_, R, t, mask)) {
             // Create Initial Map
             map_manager_->initializeMap(last_frame_, current_frame, matches, R, t);
             
             // Update State
             stage_ = Stage::TRACKING;
             last_frame_ = current_frame;
             
             // Get pose set by initializeMap
             // Oops, initializeMap sets KeyFrame pose, but we should also set current_frame pose?
             // KeyFrame copied pose from Frame, or set it?
             // initializeMap creates KFs. We should sync current_frame pose.
             // We can fetch it from map_manager or re-calculate.
             // For now, let's assume initializeMap sets poses in KFs.
             // We need to set current_frame->pose so we can track next.
             // KF2 corresponds to current_frame. 
             // We can just grab the last KF added?
             auto kfs = map_manager_->getKeyFrames();
             if(!kfs.empty()) {
                current_frame->setPose(kfs.back()->getPose());
                last_pose_ = current_frame->getPose();
             }
             
             std::cout << "[Frontend] Map Initialized! Switched to TRACKING." << std::endl;
             return true;
        } else {
             std::cout << "[Frontend] Init failed (pose)." << std::endl;
        }
        return false;
    }

    // --- 2. Tracking Case (Local Map) ---
    if (stage_ == Stage::TRACKING) {
        
        // A. Predict Pose (Constant Velocity)
        // velocity_ = last_pose_ * last_frame_pose_inverse? 
        // Simple: Pose_new = Pose_old * Velocity
        gtsam::Pose3 predicted_pose = last_pose_ * velocity_;
        current_frame->setPoseGuess(predicted_pose);
        
        // B. Track Local Map (Projection)
        std::vector<MapPoint::Ptr> local_map = map_manager_->getLocalMapPoints();
        int n_matches = feature_matcher_->matchProjection(current_frame, local_map);
        
        // std::cout << "[Frontend] Tracked " << n_matches << " MapPoints." << std::endl;
        
        if (n_matches < 10) {
            std::cout << "[Frontend] Tracking LOST! (Only " << n_matches << " matches)" << std::endl;
            // Handle lost?
            // velocity_ = identity?
            return false;
        }
        
        // C. Pose Refinement
        if (pose_estimator_->estimateRefined(current_frame)) {
             // Update Velocity
             gtsam::Pose3 curr_pose = current_frame->getPose();
             // velocity = last_pose.inverse() * curr_pose; (T_k-1_k)
             // velocity = T_w_old.inv * T_w_new
             velocity_ = last_pose_.inverse() * curr_pose; 
             
             last_pose_ = curr_pose;
             
             // D. KeyFrame Selection
             if (keyframe_selector_->check(current_frame, n_matches)) {
                 map_manager_->update(current_frame, true); // Create KF
             } else {
                 map_manager_->update(current_frame, false); // Just housekeeping?
             }
             
             last_frame_ = current_frame; // Keep for next iteration if needed
             return true;
        } else {
             std::cout << "[Frontend] Pose Refinement Failed." << std::endl;
             return false;
        }
    }

    return false;
}

} // namespace frontend