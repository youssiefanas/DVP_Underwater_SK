#include "frontend/Frame.hpp"

namespace frontend {

size_t Frame::next_id_ = 0;

Frame::Frame(size_t id, double timestamp, const cv::Mat &image)
    : id_(id), timestamp_(timestamp), image_(image.clone()) {}

Frame::Ptr Frame::createFrame(const cv::Mat &image, double timestamp) {
  Frame::Ptr new_frame(new Frame(next_id_++, timestamp, image));
  return new_frame;
}

void Frame::setFeatures(const std::vector<cv::KeyPoint> &kps,
                        const cv::Mat &des) {
  keypoints_ = kps;
  descriptors_ = des;
  // Maintain invariant: map_points_ has the same size as keypoints_
  map_points_.assign(keypoints_.size(), nullptr);
}

size_t Frame::getId() const { return id_; }
double Frame::getTimestamp() const { return timestamp_; }
const cv::Mat &Frame::getImage() const { return image_; }

const Pose3d &Frame::getPose() const { return pose_; }
void Frame::setPose(const Pose3d &pose) { pose_ = pose; }

const std::vector<cv::KeyPoint> &Frame::getKeypoints() const {
  return keypoints_;
}
const cv::Mat &Frame::getDescriptors() const { return descriptors_; }

const std::vector<MapPoint::Ptr> &Frame::getMapPoints() const {
  return map_points_;
}
std::vector<MapPoint::Ptr> &Frame::accessMapPoints() { return map_points_; }

void Frame::ensureMapPointVectorSized(size_t n) {
  if (map_points_.size() < n) {
    map_points_.resize(n, nullptr);
  }
}

} // namespace frontend
