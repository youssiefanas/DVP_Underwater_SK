#pragma once

#include <vector>
#include <list>
#include <memory>
#include <mutex>

#include "Frame.hpp"
#include "KeyFrame.hpp"
#include "MapPoint.hpp"

namespace frontend {

class LocalMapManager {
public:
    using Ptr = std::shared_ptr<LocalMapManager>;

    LocalMapManager(size_t max_keyframes = 10);
    ~LocalMapManager() = default;

    /**
     * @brief Process a frame and update the local map.
     * 
     * @param frame Current frame (tracked).
     * @param create_new_kf If true, promote this frame to KeyFrame.
     */
    void update(Frame::Ptr frame, bool create_new_kf);

    // --- Accessors for Tracking ---
    
    std::vector<MapPoint::Ptr> getLocalMapPoints() const;
    std::vector<KeyFrame::Ptr> getKeyFrames() const;
    
    // Reset map (for initialization)
    void reset();

    // Init map with first two frames (stereo/monocular initialization)
    void initializeMap(Frame::Ptr frame1, Frame::Ptr frame2, 
                       const std::vector<cv::DMatch>& matches,
                       const cv::Mat& R, const cv::Mat& t);

private:
    void addKeyFrame(KeyFrame::Ptr kf);
    void cullKeyFrames();
    void createMapPoints(KeyFrame::Ptr kf, Frame::Ptr frame);

private:
    size_t max_keyframes_;
    
    std::list<KeyFrame::Ptr> keyframes_; // Sliding window of KeyFrames
    std::vector<MapPoint::Ptr> all_map_points_; // All points in local map
    
    mutable std::mutex mutex_;
};

} // namespace frontend
