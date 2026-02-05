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
    map_ = std::make_shared<Map>(); 
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
    std::vector<cv::DMatch> matches_used; // matches used for pose estimation
    for (const auto& m : matches) {
        points_prev.push_back(last_frame_->getKeypoints()[m.queryIdx].pt);
        points_curr.push_back(current_frame->getKeypoints()[m.trainIdx].pt);
        matches_used.push_back(m); // add match to matches_used
    }

      cv::Mat R, t, mask;
      if (pose_estimator_->estimate(points_prev, points_curr, K_, R, t, mask)) {

        std::vector<cv::Point2f> inliers_prev, inliers_curr;
        std::vector<cv::DMatch> inliers_matches;

        int inliers_count = 0;
        for (int i=0; i<mask.rows; i++){
            if (mask.at<uchar>(i) == 1){
                inliers_count++;
                inliers_prev.push_back(points_prev[i]);
                inliers_curr.push_back(points_curr[i]);
                inliers_matches.push_back(matches_used[i]);
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

        // Add 3D points to map
        
        // validation params
        const double max_range = 50.0;
        const double min_parallax_deg = 1.0; // parallax is the angle between the lines of sight of the two cameras
        const double min_parallax_rad = min_parallax_deg * M_PI / 180.0;

        size_t created = 0;

        for (size_t i = 0; i < points_3d.size(); i++) {
            const cv::Point3f& p = points_3d[i];

            cv::Mat pt= (cv::Mat_<double>(3,1) << p.x, p.y, p.z);

            double z1 = pt.at<double>(2);
            if (z1<=0) continue;

            cv::Mat pt_2 = R * pt + t;
            double z2 = pt_2.at<double>(2);
            if (z2<=0) continue;

            // Distance check (world / camera1 frame)
            double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            if (dist > max_range) continue;

            // Parallax check: compare bearings from both cameras
            cv::Mat b1 = pt / cv::norm(pt);
            cv::Mat b2 = pt_2 / cv::norm(pt_2);
            double dot = b1.dot(b2);
            // clamp dot
            if (dot > 1.0) dot = 1.0;
            if (dot < -1.0) dot = -1.0;
            double parallax = std::acos(dot);
            if (parallax < min_parallax_rad) continue;

            // Create MapPoint and add to map
            gtsam::Point3 p_last (p.x, p.y, p.z);
            gtsam::Point3 p_world = T_last_last.transformFrom(p_last);

            Eigen::Vector3d pose_world_eigen(p_world.x(), p_world.y(), p_world.z());
            MapPoint::Ptr mp = MapPoint::create(pose_world_eigen);

            // observations: map indices are from match elements
            const cv::DMatch& m = inliers_matches[i];
            size_t idx_last = m.queryIdx;
            size_t idx_curr = m.trainIdx;


            // ensure the map_points vectors are sized
            last_frame_->ensureMapPointVectorSized(last_frame_->getKeypoints().size());
            current_frame->ensureMapPointVectorSized(current_frame->getKeypoints().size());

            mp->addObservation(last_frame_, idx_last);
            mp->addObservation(current_frame, idx_curr);

            last_frame_->accessMapPoints()[idx_last] = mp;
            current_frame->accessMapPoints()[idx_curr] = mp;
            if (!last_frame_->getDescriptors().empty()) {
                mp->descriptor_ = last_frame_->getDescriptors().row((int)idx_last).clone();
            }

            // Insert to global map
            map_->addMapPoint(mp);
            created++;
        }

                // compute some sanity stats and log
        size_t total_mp = map_->getMapPoints().size();
        size_t sum_obs = 0;
        for (const auto& mp : map_->getMapPoints()) {
            sum_obs += mp->observations_.size();
        }
        double avg_obs = (total_mp > 0) ? double(sum_obs)/double(total_mp) : 0.0;

        std::cout << "[Frontend] Inliers: " << inliers_count << "/" << matches.size() 
                  << " | Triangulated: " << points_3d.size()
                  << " | Created MapPoints: " << created
                  << " | MapPoints total: " << total_mp
                  << " | Avg obs/mp: " << avg_obs
                  << std::endl;
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

Frame::Ptr VisualFrontend::getLatestFrame() const {
    return last_frame_;
}

} // namespace frontend