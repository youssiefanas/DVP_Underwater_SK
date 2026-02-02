#pragma once

#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <atomic>

namespace frontend {

class Viewer {
public:
    using Ptr = std::shared_ptr<Viewer>;

    Viewer();
    ~Viewer();

    void show(const cv::Mat& image);
    void spin();
    void stop();

private:
    std::thread display_thread_;
    std::mutex frame_mutex_;
    cv::Mat current_frame_;
    std::atomic<bool> new_frame_available_;
    std::atomic<bool> stop_requested_;
};

} // namespace frontend
