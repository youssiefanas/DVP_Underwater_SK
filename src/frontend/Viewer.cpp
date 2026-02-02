#include "frontend/Viewer.hpp"
#include <chrono>

namespace frontend {

Viewer::Viewer() : new_frame_available_(false), stop_requested_(false) {
    display_thread_ = std::thread(&Viewer::spin, this);
}

Viewer::~Viewer() {
    stop();
}

void Viewer::stop() {
    stop_requested_ = true;
    if (display_thread_.joinable()) {
        display_thread_.join();
    }
}

void Viewer::show(const cv::Mat& image) {
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        image.copyTo(current_frame_);
        new_frame_available_ = true;
    }
}

void Viewer::spin() {
    while (!stop_requested_) {
        cv::Mat frame_to_show;
        bool should_show = false;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (new_frame_available_) {
                current_frame_.copyTo(frame_to_show);
                new_frame_available_ = false;
                should_show = true;
            }
        }

        if (should_show && !frame_to_show.empty()) {
            cv::imshow("DV SLAM Viewer", frame_to_show);
            cv::waitKey(1); // Process events
        }

        // Sleep a bit to avoid hogging CPU if no new frame
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    cv::destroyWindow("DV SLAM Viewer");
}

} // namespace frontend
