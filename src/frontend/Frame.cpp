#include "frontend/Frame.hpp"

namespace frontend {

// Initialize static factory ID counter
static size_t current_frame_id = 0;

Frame::Frame(size_t id, double timestamp, const cv::Mat& image)
    : id_(id), timestamp_(timestamp), image_(image) {
    // You can add image cloning here if you want to ensure the Frame owns the data
    // image_ = image.clone(); 
    pose_ = gtsam::Pose3();
}

Frame::Ptr Frame::createFrame(cv::Mat image, double timestamp) {
    // Create the new frame with an incremented ID
    Frame::Ptr new_frame(new Frame(current_frame_id, timestamp, image));
    current_frame_id++;
    return new_frame;
}

void Frame::setFeatures(const std::vector<cv::KeyPoint>& kps, const cv::Mat& des) {
    keypoints_ = kps;
    descriptors_ = des;
    // Maintain invariant: map_points_ has the same size as keypoints_
    map_points_.assign(keypoints_.size(), nullptr); // Initialize with nullptrs
}

// Getters
size_t Frame::getId() const { return id_; }
double Frame::getTimestamp() const { return timestamp_; }
const cv::Mat& Frame::getImage() const { return image_; }

// Pose Getter/Setter
// Pass by const reference to avoid copying
const gtsam::Pose3& Frame::getPose() const { return pose_; }
void Frame::setPose(const gtsam::Pose3& pose) { pose_ = pose; }

// Access features (const reference to avoid copying)
const std::vector<cv::KeyPoint>& Frame::getKeypoints() const { return keypoints_; }
const cv::Mat& Frame::getDescriptors() const { return descriptors_; }

// MapPoint accessor
const std::vector<MapPoint::Ptr>& Frame::getMapPoints() const { return map_points_; }
std::vector<MapPoint::Ptr>& Frame::accessMapPoints() { return map_points_; }
void Frame::ensureMapPointVectorSized(size_t n) { map_points_.assign(n, nullptr); }

} // namespace frontend