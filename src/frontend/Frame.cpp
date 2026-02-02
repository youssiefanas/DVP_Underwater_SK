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
    // 1. Create the new frame with an incremented ID
    Frame::Ptr new_frame(new Frame(current_frame_id, timestamp, image));
    
    // 2. Increment id for the next frame
    current_frame_id++;
    
    return new_frame;
}

void Frame::setFeatures(const std::vector<cv::KeyPoint>& kps, const cv::Mat& des) {
    keypoints_ = kps;
    descriptors_ = des;
    resizeObservations(kps.size());
}

} // namespace frontend