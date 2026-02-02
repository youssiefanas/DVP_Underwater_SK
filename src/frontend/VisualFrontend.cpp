#include "frontend/VisualFrontend.hpp"
#include "dv_slam/utility.hpp"
#include <iostream> 

namespace frontend {

VisualFrontend::VisualFrontend(const ORBParams& params, const cv::Mat& K) : stage_(Stage::NO_IMAGES_YET) {
    feature_extractor_ = std::make_shared<FeatureExtractor>(params);
    viewer_ = std::make_shared<Viewer>();
    feature_matcher_ = std::make_shared<FeatureMatcher>(params.matcher_type);
    pose_estimator_ = std::make_shared<PoseEstimator>();
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
    bool success = process(frame);

    // Visualization
    if (viewer_) {
        cv::Mat img_out;
        cv::drawKeypoints(frame->getImage(), frame->getKeypoints(), img_out);
        // Draw tracked points in green
        const auto& points = frame->getKeypoints();
        for (const auto& p : points) {
            cv::circle(img_out, p.pt, 2, cv::Scalar(0, 255, 0), -1);
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
    
    // --- 1. Initialization Case 
    if (!last_frame_) {
      std::cout << "[Frontend] First frame received. Initializing..."
                << std::endl;
      last_frame_ = current_frame;
      current_frame->setPose(gtsam::Pose3()); // Identity pose for the first frame
      return false;
    } 

    std::vector<cv::DMatch> matches = feature_matcher_->match(last_frame_, current_frame);

    if (matches.size() < 20) {
        std::cout << "[Frontend] Not enough matches found. Skipping frame."
                  << std::endl;
        return false;
    }

    std::vector<cv::Point2f> points_prev, points_curr;
    for (const auto& m : matches) {
        points_prev.push_back(last_frame_->getKeypoints()[m.queryIdx].pt);
        points_curr.push_back(current_frame->getKeypoints()[m.trainIdx].pt);
    }

      cv::Mat R, t, mask;
      if (pose_estimator_->estimate(points_prev, points_curr, K_, R, t, mask)) {

        std::vector<cv::Point2f> inliers_prev, inliers_curr;

        int inliers_count = 0;
        for (int i=0; i<mask.rows; i++){
            if (mask.at<uchar>(i) == 1){
                inliers_count++;
                inliers_prev.push_back(points_prev[i]);
                inliers_curr.push_back(points_curr[i]);
            }
        }
        if (inliers_count < 20){
            std::cout << "[Frontend] Not enough inliers. Skipping frame." << std::endl;
            return false;
        }

        std::vector<cv::Point3f> points_3d;
        pose_estimator_->triangulate(inliers_prev, inliers_curr, K_, R, t, points_3d);

        gtsam::Pose3 T_last_curr = cvToGtsam(R, t);
        gtsam::Pose3 T_last_last = last_frame_->getPose();
        gtsam::Pose3 T_w_c = T_last_last * T_last_curr;
        current_frame->setPose(T_w_c);

        // Log Output for Verification
        std::cout << "[Frontend] Inliers: " << inliers_count << "/" << matches.size() 
                  << " | 3D Points: " << points_3d.size() << std::endl;
        std::cout << "Pose: " << T_w_c.translation().transpose() << std::endl;

        // Shift Window
        last_frame_ = current_frame;
        return true;
    } else {
        std::cout << "[Frontend] relative pose estimation failed." << std::endl;
        return false;
    }
}

} // namespace frontend