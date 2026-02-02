#include "frontend/KeyframeSelector.hpp"
#include <iostream>

namespace frontend {

KeyframeSelector::KeyframeSelector(int min_tracked_points, int frames_since_last_kf_thresh)
    : min_tracked_points_(min_tracked_points), 
      frames_since_last_kf_thresh_(frames_since_last_kf_thresh) {}

bool KeyframeSelector::check(Frame::Ptr current_frame, int tracked_map_points_count) {
    frames_since_last_kf_++;

    // 1. Quality Check: If tracking is getting weak, we need a new KF
    bool need_kf_quality = tracked_map_points_count < min_tracked_points_;
    
    // 2. Interval Check: Don't span too long without a KF (drift accumulation)
    bool need_kf_time = frames_since_last_kf_ >= frames_since_last_kf_thresh_;

    // 3. Motion Check (Placeholder)
    // Could check relative motion since last KF > threshold
    
    // Condition: 
    // - Must have tracked SOME points (don't KF if tracking lost completely)
    // - AND (Quality weak OR Time passed)
    
    if (tracked_map_points_count > 5 && (need_kf_quality || need_kf_time)) {
        frames_since_last_kf_ = 0;
        return true;
    }

    return false;
}

} // namespace frontend
