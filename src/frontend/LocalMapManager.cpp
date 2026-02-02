#include "frontend/LocalMapManager.hpp"
#include <iostream>
#include <algorithm> // for remove_if
#include <opencv2/calib3d.hpp> // Added for triangulatePoints

namespace frontend {

LocalMapManager::LocalMapManager(size_t max_keyframes) 
    : max_keyframes_(max_keyframes) {}

void LocalMapManager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.clear();
    all_map_points_.clear();
}

std::vector<MapPoint::Ptr> LocalMapManager::getLocalMapPoints() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return all_map_points_;
}

std::vector<KeyFrame::Ptr> LocalMapManager::getKeyFrames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<KeyFrame::Ptr> kfs_vec(keyframes_.begin(), keyframes_.end());
    return kfs_vec;
}

void LocalMapManager::update(Frame::Ptr frame, bool create_new_kf) {
    if (!frame) return;

    if (create_new_kf) {
        std::cout << "[LocalMapManager] Creating new KeyFrame from Frame " << frame->getId() << std::endl;
        
        // 1. Create KeyFrame
        KeyFrame::Ptr new_kf = std::make_shared<KeyFrame>(frame);
        
        // 2. Add to Map
        addKeyFrame(new_kf);
        
        // 3. Create new MapPoints for features that don't have them yet 
        // (Triangulate with previous KF! - Simplified here: just promote untracked features to 3D if possible? 
        //  Actually, in Monocular, new MapPoints are created by triangulating matches between Frames.
        //  Here we might only add points that are already triangulated/initialized.
        //  BUT logic: If we just initialized, we have structure. 
        //  If we are tracking, we add a KF. 
        //  New MapPoints creation typically requires searching for matches in OTHER KFs.
        //  For this "Canonical Dataflow" prototype, we assume Initialization creates points,
        //  and we might defer new point creation to a specific "Mapping" step.
        //  HOWEVER, to keep tracking alive, we need to add points.
        //  Lets assume for now: KeyFrame just adopts existing tracked points.
        //  New point creation (Triangulation) is complex. 
        //  We will skip simplified new point creation here unless we have matches to prev KF.
        
        //  Wait, if we don't create new points, the map eventually dies.
        //  So we MUST create points.
        //  Simplified strategy: 
        //  If we have a previous KF, match new features, triangulate, add MapPoints.
        
        //  Access previous KF
        //  (Need to handle concurrency with getKeyFrames)
    }
}

void LocalMapManager::initializeMap(Frame::Ptr frame1, Frame::Ptr frame2, 
                                    const std::vector<cv::DMatch>& matches,
                                    const cv::Mat& R, const cv::Mat& t) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Create KeyFrames
    KeyFrame::Ptr kf1 = std::make_shared<KeyFrame>(frame1);
    KeyFrame::Ptr kf2 = std::make_shared<KeyFrame>(frame2);
    
    // Set Pose for KF1 (Identity) and KF2 (R, t)
    // Assume T_w_c convention
    kf1->setPose(gtsam::Pose3());
    
    // R, t from estimate() are usually Frame2 relative to Frame1 (T_1_2)?
    // recoverPose returns R, t such that x2 = R*x1 + t. 
    // This is P_2 = T_2_1 * P_1.  So R,t is T_2_1 (or T_c2_c1).
    // If Global 1 is Identity, Pose 2 = T_w_c2 = T_1_2.inverse()? No.
    // If Pose1 is origin (Identity). P_w = P_c1.
    // P_c2 = R*P_c1 + t.
    // So T_c2_w = (R, t). 
    // Pose 2 (T_w_c2) = T_c2_w.inverse().
    
    Eigen::Matrix3d R_eigen;
    Eigen::Vector3d t_eigen;
    for(int i=0; i<3; i++) {
         for(int j=0; j<3; j++) R_eigen(i,j) = R.at<double>(i,j);
         t_eigen(i) = t.at<double>(i);
    }
    
    gtsam::Rot3 rot(R_eigen);
    gtsam::Point3 trans(t_eigen);
    gtsam::Pose3 T_c2_w(rot, trans);
    
    kf2->setPose(T_c2_w.inverse());
    
    // Create MapPoints
    const cv::Mat& K = frame1->getCamera();
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);
    
    // Projection matrices for triangulation
    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F); // P1 = K[I|0] -> simplified, K applied later
    // Actually triangulatePoints takes 3x4 projection matrices P = K * [R|t]
    
    cv::Mat P1_full = cv::Mat::zeros(3, 4, CV_64F);
    K.copyTo(P1_full(cv::Rect(0,0,3,3)));
    
    cv::Mat P2_full = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat Rt = cv::Mat::zeros(3, 4, CV_64F);
    R.copyTo(Rt(cv::Rect(0,0,3,3)));
    t.copyTo(Rt(cv::Rect(3,0,1,3))); // R,t is T_c2_c1 (T_c2_w)
    P2_full = K * Rt;
    
    // Collect points
    std::vector<cv::Point2f> pts1, pts2;
    for(const auto& m : matches) {
        pts1.push_back(frame1->getKeypoints()[m.queryIdx].pt);
        pts2.push_back(frame2->getKeypoints()[m.trainIdx].pt);
    }
    
    cv::Mat points4D;
    cv::triangulatePoints(P1_full, P2_full, pts1, pts2, points4D);
    
    for(size_t i=0; i<matches.size(); i++) {
        cv::Mat x = points4D.col(i);
        x /= x.at<float>(3); // Normalize w
        
        if (std::abs(x.at<float>(3)) < 1e-6) continue;
        
        Eigen::Vector3d pos(x.at<float>(0), x.at<float>(1), x.at<float>(2));
        
        // Create MapPoint
        MapPoint::Ptr mp = std::make_shared<MapPoint>(pos);
        mp->setDescriptor(frame1->getDescriptors().row(matches[i].queryIdx)); // Clone?
        
        all_map_points_.push_back(mp);
        
        // Link to KeyFrames
        kf1->addMapPoint(mp, matches[i].queryIdx);
        kf2->addMapPoint(mp, matches[i].trainIdx); // Assuming trainIdx matches kf2 structure
        
        // Add observation ref
        mp->addObservation(kf1, matches[i].queryIdx);
        mp->addObservation(kf2, matches[i].trainIdx);
    }
    
    keyframes_.push_back(kf1);
    keyframes_.push_back(kf2);
    
    std::cout << "[LocalMapManager] Map Initialized. KFs: 2, MapPoints: " << all_map_points_.size() << std::endl;
}

void LocalMapManager::addKeyFrame(KeyFrame::Ptr kf) {
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.push_back(kf);
    cullKeyFrames();
}

void LocalMapManager::cullKeyFrames() {
    while (keyframes_.size() > max_keyframes_) {
        // Simple sliding window: remove oldest
        // Ideally should remove redundant ones
        keyframes_.pop_front();
    }
}

} // namespace frontend
