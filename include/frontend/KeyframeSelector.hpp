#pragma once

#include "Frame.hpp"

namespace frontend {

class KeyframeSelector {
public:
    KeyframeSelector(int min_tracked_points = 20, int frames_since_last_kf_thresh = 20);
    ~KeyframeSelector() = default;

    /**
     * @brief Decide if the current frame should be a KeyFrame.
     * 
     * @param current_frame The current tracked frame.
     * @param tracked_map_points_count Number of MapPoints successfully tracked.
     * @return true if a new KeyFrame is needed.
     */
    bool check(Frame::Ptr current_frame, int tracked_map_points_count);

    void reset() {
        frames_since_last_kf_ = 0;
    }

private:
    int min_tracked_points_;
    int frames_since_last_kf_thresh_;
    int frames_since_last_kf_ = 0;
};

} // namespace frontend
