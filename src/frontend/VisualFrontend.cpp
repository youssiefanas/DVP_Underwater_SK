#include "frontend/VisualFrontend.hpp"
#include <iostream> 

namespace frontend {

VisualFrontend::VisualFrontend(const ORBParams& params, const cv::Mat& K) : stage_(Stage::NO_IMAGES_YET) {
    feature_extractor_ = std::make_shared<FeatureExtractor>(params);
    viewer_ = std::make_shared<Viewer>();
    feature_matcher_ = std::make_shared<FeatureMatcher>(params.matcher_type);
    K_ = K.clone();
}

bool VisualFrontend::handleImage(
    const cv::Mat& gray_image,
    double timestamp)
{
    // Create frame (data ownership)
    Frame::Ptr frame = Frame::createFrame(gray_image, timestamp);

    // Feature extraction (annotation)
    extractFeatures(frame);
    
    // Visualization
    if (viewer_) {
        cv::Mat img_out;
        cv::drawKeypoints(frame->getImage(), frame->getKeypoints(), img_out);
        viewer_->show(img_out);
    }

    // Core frontend logic
    return process(frame);
}

void VisualFrontend::extractFeatures(Frame::Ptr frame)
{
    feature_extractor_->extract(*frame);
}

bool VisualFrontend::process(Frame::Ptr current_frame) {
    
    // --- 1. Initialization Case (First Frame) ---
    if (stage_ == Stage::NO_IMAGES_YET) {
        std::cout << "[Frontend] First frame received. Initializing..." << std::endl;
        
        // Store this frame as the "previous" frame for the next iteration
        last_frame_ = current_frame;
        
        // Advance state
        stage_ = Stage::INITIALIZED;
        
        // Return false because we can't do odometry with just 1 frame
        return false;
    }

    // --- 2. Tracking Case (Subsequent Frames) ---
    if (stage_ == Stage::INITIALIZED || stage_ == Stage::TRACKING) {
        // std::cout << "[Frontend] Processing Frame ID: " << current_frame->getId() << std::endl;

        // Match features between previous and current frame
        std::vector<cv::DMatch> matches = feature_matcher_->match(last_frame_, current_frame);

        if (matches.size() < 10) {
            std::cout << "[Frontend] Not enough matches found. Skipping frame." << std::endl;
            return false;
        }

        std::vector<cv::Point2f> points_prev, points_curr;
        for (const auto& m : matches) {
            points_prev.push_back(last_frame_->getKeypoints()[m.queryIdx].pt);
            points_curr.push_back(current_frame->getKeypoints()[m.trainIdx].pt);
        }
        // 3. Geometric Verification (RANSAC)
        // We calculate the Essential Matrix (E). This requires the camera intrinsics (K).
        // Assuming you have a cv::Mat K (Intrinsics Matrix)
        cv::Mat mask; // This will store 0 for outliers, 1 for inliers
        cv::Mat E = cv::findEssentialMat(points_prev, points_curr, K_, cv::RANSAC, 0.999, 1.0, mask);

            // 4. Filter the matches (Keep only inliers)
            std::vector<cv::DMatch> good_matches_geometric;
            for (int i = 0; i < mask.rows; i++) {
                if (mask.at<uchar>(i) == 1) {
                    good_matches_geometric.push_back(matches[i]);
                }
            }

            std::cout << "[Frontend] Raw Matches: " << matches.size() 
                    << " -> Geometric Inliers: " << good_matches_geometric.size() << std::endl;

        // --- Critical Step: Shift the "Window" ---
        // The current frame becomes the old frame for the next iteration
        last_frame_ = current_frame;
        stage_ = Stage::TRACKING;
        return true;
    }

    return false;
}

} // namespace frontend