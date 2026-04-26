#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace frontend {

/**
 * @brief Time-keyed ring buffer of IMU world-frame attitudes.
 *
 * Image and IMU callbacks may run on different threads under a multi-threaded
 * executor; access is mutex-guarded. Old samples are evicted on push so the
 * buffer stays bounded.
 */
class ImuBuffer {
public:
    struct Sample {
        double t;
        Eigen::Matrix3d R_world_imu;
    };

    void push(double t, const Eigen::Matrix3d &R_world_imu);

    /// Closest sample within @p max_dt of @p t, or nullopt.
    std::optional<Eigen::Matrix3d> queryAttitudeAt(double t,
                                                    double max_dt) const;

    size_t size() const;

private:
    static constexpr double kMaxBufferAgeSeconds = 5.0;

    mutable std::mutex mutex_;
    std::deque<Sample> samples_;
};

inline void ImuBuffer::push(double t, const Eigen::Matrix3d &R_world_imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.push_back({t, R_world_imu});
    const double cutoff = t - kMaxBufferAgeSeconds;
    while (!samples_.empty() && samples_.front().t < cutoff) {
        samples_.pop_front();
    }
}

inline std::optional<Eigen::Matrix3d>
ImuBuffer::queryAttitudeAt(double t, double max_dt) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto best = samples_.end();
    double best_dt = max_dt;
    for (auto it = samples_.begin(); it != samples_.end(); ++it) {
        const double dt = std::abs(it->t - t);
        if (dt <= best_dt) {
            best_dt = dt;
            best = it;
        }
    }
    if (best == samples_.end()) return std::nullopt;
    return best->R_world_imu;
}

inline size_t ImuBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

} // namespace frontend
